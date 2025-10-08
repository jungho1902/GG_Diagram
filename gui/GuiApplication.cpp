#include "GuiApplication.hpp"

#include "math_utils.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include <sstream>
#include <utility>
#include <stdexcept>
#include <string>

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

namespace gg::gui {

namespace {
void glfwErrorCallback(int error, const char* description) {
    std::fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

ImVec2 mapToCanvas(const Point2D& point, const ImVec2& origin, const ImVec2& size, float axisRange) {
    if (axisRange <= 0.0f) {
        return origin;
    }
    float minAxis = -axisRange;
    float maxAxis = axisRange;
    float xNorm = (static_cast<float>(point.x) - minAxis) / (maxAxis - minAxis);
    float yNorm = (static_cast<float>(point.y) - minAxis) / (maxAxis - minAxis);
    xNorm = std::clamp(xNorm, 0.0f, 1.0f);
    yNorm = std::clamp(yNorm, 0.0f, 1.0f);

    float x = origin.x + xNorm * size.x;
    float y = origin.y + (1.0f - yNorm) * size.y;
    return ImVec2{x, y};
}
}  // namespace

GuiApplication::GuiApplication() {
    initWindow();
    initImGui();
    pipeline_.setDatasetPlaybackSpeed(datasetPlayback_);
}

GuiApplication::~GuiApplication() {
    shutdown();
}

void GuiApplication::initWindow() {
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    window_ = glfwCreateWindow(1280, 720, "G-G Diagram Visualizer", nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    glewExperimental = GL_TRUE;
    GLenum glewStatus = glewInit();
    if (glewStatus != GLEW_OK) {
        std::string message = "Failed to initialize GLEW: ";
        message += reinterpret_cast<const char*>(glewGetErrorString(glewStatus));
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        throw std::runtime_error(message);
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void GuiApplication::initImGui() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 150");
}

void GuiApplication::shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

void GuiApplication::run() {
    while (!glfwWindowShouldClose(window_) && !shouldClose_) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        pipeline_.update(ImGui::GetIO().DeltaTime);

        drawControls();
        drawPlot();
        drawParameters();
        drawAnalytics();

        ImGui::Render();
        int display_w = 0;
        int display_h = 0;
        glfwGetFramebufferSize(window_, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window_);
    }
}

void GuiApplication::drawControls() {
    ImGui::Begin("Simulation Controls");

    if (ImGui::Button(pipeline_.isRunning() ? "Stop Simulation" : "Start Simulation")) {
        pipeline_.toggle();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset Data")) {
        pipeline_.reset();
        exportStatus_.clear();
        datasetStatus_.clear();
        analyticsStatus_.clear();
        monteCarloStatus_.clear();
    }

    ImGui::Separator();

    SimulationStats stats = pipeline_.stats();
    ImGui::Text("Current Gx: %.3f g", stats.gx);
   ImGui::Text("Current Gy: %.3f g", stats.gy);
   ImGui::Text("Processed Points: %zu", stats.totalPoints);
   ImGui::Text("Steady-State Points: %zu", stats.steadyPoints);

    ImGui::Separator();

    bool datasetToggle = useDataset_;
    if (ImGui::Checkbox("Use Dataset Playback", &datasetToggle)) {
        useDataset_ = datasetToggle;
        pipeline_.setUseDataset(useDataset_);
    }

    ImGui::InputText("Dataset CSV Path", &datasetPath_);
    if (ImGui::Button("Load Dataset")) {
        if (pipeline_.loadDatasetFromCsv(datasetPath_)) {
            datasetStatus_ = "Loaded dataset: " + std::to_string(pipeline_.datasetSize()) + " samples";
            if (useDataset_) {
                pipeline_.setUseDataset(true);
            }
            analyticsStatus_.clear();
            monteCarloStatus_.clear();
        } else {
            datasetStatus_ = "Failed to load dataset";
        }
    }
    if (!datasetStatus_.empty()) {
        ImGui::TextWrapped("%s", datasetStatus_.c_str());
    }

    float playback = datasetPlayback_;
    if (ImGui::SliderFloat("Playback Speed (x)", &playback, 0.1f, 4.0f, "%.2fx")) {
        datasetPlayback_ = playback;
        pipeline_.setDatasetPlaybackSpeed(playback);
    }

    if (pipeline_.useDataset() && pipeline_.hasDataset()) {
        ImGui::Text("Dataset points: %zu", pipeline_.datasetSize());
        ImGui::Text("Playback status: %s", pipeline_.datasetFinished() ? "completed" : (pipeline_.isRunning() ? "running" : "ready"));
    }

    ImGui::Separator();

    int ticks = tickCount_;
    if (ImGui::InputInt("Tick Count", &ticks)) {
        tickCount_ = std::max(2, ticks);
    }

    ImGui::InputText("SVG Output File", &exportPath_);
    if (ImGui::Button("Export SVG")) {
        if (exportSvg(exportPath_)) {
            exportStatus_ = "Exported SVG to " + exportPath_;
        } else {
            exportStatus_ = "Failed to export SVG";
        }
    }
    if (!exportStatus_.empty()) {
        ImGui::TextWrapped("%s", exportStatus_.c_str());
    }

    ImGui::Separator();

    auto thresholds = pipeline_.thresholds();
    bool changed = false;

    float jerk = static_cast<float>(thresholds.jerk);
    if (ImGui::SliderFloat("Jerk Threshold (m/s^3)", &jerk, 0.05f, 1.0f, "%.2f")) {
        thresholds.jerk = jerk;
        changed = true;
    }

    float angularAccelDeg = static_cast<float>(thresholds.angularAcceleration * 180.0 / 3.14159265358979323846);
    if (ImGui::SliderFloat("Angular Acceleration (deg/s^2)", &angularAccelDeg, 0.5f, 20.0f, "%.1f")) {
        thresholds.angularAcceleration = angularAccelDeg * 3.14159265358979323846 / 180.0;
        changed = true;
    }

    float dvdt = static_cast<float>(thresholds.velocityDerivativeRms);
    if (ImGui::SliderFloat("Velocity d/dt RMS (m/s^3)", &dvdt, 0.1f, 1.0f, "%.2f")) {
        thresholds.velocityDerivativeRms = dvdt;
        changed = true;
    }

    float yawDerivativeDeg = static_cast<float>(thresholds.yawDerivativeRms * 180.0 / 3.14159265358979323846);
    if (ImGui::SliderFloat("Yaw Rate d/dt RMS (deg/s^2)", &yawDerivativeDeg, 0.2f, 10.0f, "%.2f")) {
        thresholds.yawDerivativeRms = yawDerivativeDeg * 3.14159265358979323846 / 180.0;
        changed = true;
    }

    float windowSeconds = static_cast<float>(thresholds.windowDuration);
    if (ImGui::SliderFloat("Window Duration (s)", &windowSeconds, 0.05f, 1.0f, "%.2f")) {
        thresholds.windowDuration = windowSeconds;
        changed = true;
    }

    if (changed) {
        pipeline_.setThresholds(thresholds);
    }

    ImGui::End();
}

void GuiApplication::drawPlot() {
    ImGui::Begin("G-G Diagram");
    ImVec2 canvasPos = ImGui::GetCursorScreenPos();
    ImVec2 canvasSize = ImGui::GetContentRegionAvail();
    if (canvasSize.x < 100.0f) canvasSize.x = 100.0f;
    if (canvasSize.y < 100.0f) canvasSize.y = 100.0f;

    ImDrawList* drawList = ImGui::GetWindowDrawList();

    ImVec2 canvasEnd{canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y};
    drawList->AddRectFilled(canvasPos, canvasEnd, IM_COL32(30, 30, 35, 255));
    drawList->AddRect(canvasPos, canvasEnd, IM_COL32(80, 80, 90, 255));

    drawList->PushClipRect(canvasPos, canvasEnd, true);

    float zeroX = mapToCanvas(Point2D{0.0, 0.0}, canvasPos, canvasSize, axisRange_).x;
    float zeroY = mapToCanvas(Point2D{0.0, 0.0}, canvasPos, canvasSize, axisRange_).y;
    drawList->AddLine(ImVec2{zeroX, canvasPos.y}, ImVec2{zeroX, canvasEnd.y}, IM_COL32(120, 120, 140, 180));
    drawList->AddLine(ImVec2{canvasPos.x, zeroY}, ImVec2{canvasEnd.x, zeroY}, IM_COL32(120, 120, 140, 180));

    int ticks = std::max(2, tickCount_);
    float tickStep = (axisRange_ * 2.0f) / static_cast<float>(ticks);
    ImU32 gridColor = IM_COL32(60, 60, 70, 120);
    ImU32 tickColor = IM_COL32(160, 160, 180, 200);
    float fontSize = ImGui::GetFontSize();

    for (int i = 0; i <= ticks; ++i) {
        float value = -axisRange_ + i * tickStep;
        if (std::abs(value) > axisRange_ + 1e-3f) {
            continue;
        }

        ImVec2 verticalStart = mapToCanvas(Point2D{value, -axisRange_}, canvasPos, canvasSize, axisRange_);
        ImVec2 verticalEnd = mapToCanvas(Point2D{value, axisRange_}, canvasPos, canvasSize, axisRange_);
        drawList->AddLine(verticalStart, verticalEnd, gridColor);

        ImVec2 horizontalStart = mapToCanvas(Point2D{-axisRange_, value}, canvasPos, canvasSize, axisRange_);
        ImVec2 horizontalEnd = mapToCanvas(Point2D{axisRange_, value}, canvasPos, canvasSize, axisRange_);
        drawList->AddLine(horizontalStart, horizontalEnd, gridColor);

        ImVec2 tickPosX = mapToCanvas(Point2D{value, 0.0f}, canvasPos, canvasSize, axisRange_);
        drawList->AddLine(ImVec2{tickPosX.x, zeroY - 4.0f}, ImVec2{tickPosX.x, zeroY + 4.0f}, tickColor);

        ImVec2 tickPosY = mapToCanvas(Point2D{0.0f, value}, canvasPos, canvasSize, axisRange_);
        drawList->AddLine(ImVec2{zeroX - 4.0f, tickPosY.y}, ImVec2{zeroX + 4.0f, tickPosY.y}, tickColor);

        char label[32];
        std::snprintf(label, sizeof(label), "%0.2f", value);
        drawList->AddText(ImVec2{tickPosX.x - fontSize * 0.6f, zeroY + 6.0f}, tickColor, label);
        drawList->AddText(ImVec2{zeroX + 6.0f, tickPosY.y - fontSize * 0.5f}, tickColor, label);
    }

    drawList->AddText(ImVec2{canvasEnd.x - 70.0f, zeroY + 20.0f}, tickColor, "Gx [g]");
    drawList->AddText(ImVec2{zeroX + 10.0f, canvasPos.y + 8.0f}, tickColor, "Gy [g]");

    ImU32 processedColor = IM_COL32(160, 160, 160, 160);
    ImU32 steadyColor = IM_COL32(80, 160, 255, 220);
    ImU32 envelopeColor = IM_COL32(255, 120, 120, 240);

    for (const auto& point : pipeline_.processedPoints()) {
        ImVec2 pos = mapToCanvas(point, canvasPos, canvasSize, axisRange_);
        drawList->AddCircleFilled(pos, 2.0f, processedColor, 6);
    }

    for (const auto& point : pipeline_.steadyStatePoints()) {
        ImVec2 pos = mapToCanvas(point, canvasPos, canvasSize, axisRange_);
        drawList->AddCircleFilled(pos, 3.0f, steadyColor, 8);
    }

    const auto& hull = pipeline_.envelopeHull();
    if (hull.size() >= 2) {
        for (std::size_t i = 0; i < hull.size(); ++i) {
            const auto& a = hull[i];
            const auto& b = hull[(i + 1) % hull.size()];
            ImVec2 pa = mapToCanvas(a, canvasPos, canvasSize, axisRange_);
            ImVec2 pb = mapToCanvas(b, canvasPos, canvasSize, axisRange_);
            drawList->AddLine(pa, pb, envelopeColor, 2.0f);
        }
    }

    if (showMonteCarloSamples_ && !pipeline_.monteCarloSamples().empty()) {
        ImU32 mcSampleColor = IM_COL32(255, 240, 120, 140);
        for (const auto& point : pipeline_.monteCarloSamples()) {
            ImVec2 pos = mapToCanvas(point, canvasPos, canvasSize, axisRange_);
            drawList->AddCircleFilled(pos, 1.6f, mcSampleColor, 6);
        }
    }

    const auto& mcHull = pipeline_.monteCarloEnvelope();
    if (mcHull.size() >= 2) {
        ImU32 mcColor = IM_COL32(255, 210, 0, 220);
        for (std::size_t i = 0; i < mcHull.size(); ++i) {
            const auto& a = mcHull[i];
            const auto& b = mcHull[(i + 1) % mcHull.size()];
            ImVec2 pa = mapToCanvas(a, canvasPos, canvasSize, axisRange_);
            ImVec2 pb = mapToCanvas(b, canvasPos, canvasSize, axisRange_);
            drawList->AddLine(pa, pb, mcColor, 2.5f);
        }
    }

    drawList->PopClipRect();

    ImGui::Dummy(canvasSize);
    ImGui::End();
}

void GuiApplication::drawParameters() {
    ImGui::Begin("Parameters");

    auto processorParams = pipeline_.processorParams();
    float accelBias[3] = {static_cast<float>(processorParams.accelBias.x),
                          static_cast<float>(processorParams.accelBias.y),
                          static_cast<float>(processorParams.accelBias.z)};
    float gyroBias[3] = {static_cast<float>(processorParams.gyroBias.x),
                         static_cast<float>(processorParams.gyroBias.y),
                         static_cast<float>(processorParams.gyroBias.z)};
    float leverArm[3] = {static_cast<float>(processorParams.leverArm.x),
                         static_cast<float>(processorParams.leverArm.y),
                         static_cast<float>(processorParams.leverArm.z)};
    float gravity[3] = {static_cast<float>(processorParams.gravity.x),
                        static_cast<float>(processorParams.gravity.y),
                        static_cast<float>(processorParams.gravity.z)};
    float velocityDamping = static_cast<float>(processorParams.velocityDamping);

    bool processorChanged = false;

    if (ImGui::InputFloat3("Accel Bias (m/s^2)", accelBias, "%.4f")) {
        processorChanged = true;
    }
    if (ImGui::InputFloat3("Gyro Bias (rad/s)", gyroBias, "%.5f")) {
        processorChanged = true;
    }
    if (ImGui::InputFloat3("Lever Arm (m)", leverArm, "%.4f")) {
        processorChanged = true;
    }
    if (ImGui::InputFloat3("Gravity Vector (m/s^2)", gravity, "%.4f")) {
        processorChanged = true;
    }
    if (ImGui::InputFloat("Velocity Damping (1/s)", &velocityDamping, 0.0f, 0.0f, "%.4f")) {
        processorChanged = true;
    }

    if (processorChanged) {
        SimulationPipeline::ProcessorParams updated = processorParams;
        updated.accelBias = Vec3{static_cast<double>(accelBias[0]),
                                 static_cast<double>(accelBias[1]),
                                 static_cast<double>(accelBias[2])};
        updated.gyroBias = Vec3{static_cast<double>(gyroBias[0]),
                                static_cast<double>(gyroBias[1]),
                                static_cast<double>(gyroBias[2])};
        updated.leverArm = Vec3{static_cast<double>(leverArm[0]),
                                static_cast<double>(leverArm[1]),
                                static_cast<double>(leverArm[2])};
        updated.velocityDamping = std::max(0.0, static_cast<double>(velocityDamping));
        updated.gravity = Vec3{static_cast<double>(gravity[0]),
                               static_cast<double>(gravity[1]),
                               static_cast<double>(gravity[2])};
        pipeline_.setProcessorParams(updated);
        analyticsStatus_.clear();
        monteCarloStatus_.clear();
    }

    ImGui::Separator();

    auto analyticsParams = pipeline_.analyticsParams();
    float accelSat = static_cast<float>(analyticsParams.accelSaturation);
    float gyroSat = static_cast<float>(analyticsParams.gyroSaturation);
    int maxLag = static_cast<int>(analyticsParams.maxLagSamples);
    float allanTau = static_cast<float>(analyticsParams.allanMaxTau);
    int allanSteps = static_cast<int>(analyticsParams.allanSteps);

    bool analyticsChanged = false;

    if (ImGui::InputFloat("Accel Saturation Threshold", &accelSat, 0.0f, 0.0f, "%.2f")) {
        analyticsChanged = true;
    }
    if (ImGui::InputFloat("Gyro Saturation Threshold", &gyroSat, 0.0f, 0.0f, "%.2f")) {
        analyticsChanged = true;
    }
    if (ImGui::InputInt("Time Sync Max Lag (samples)", &maxLag)) {
        analyticsChanged = true;
    }
    if (ImGui::InputFloat("Allan Max Tau (s)", &allanTau, 0.0f, 0.0f, "%.2f")) {
        analyticsChanged = true;
    }
    if (ImGui::InputInt("Allan Steps", &allanSteps)) {
        analyticsChanged = true;
    }

    if (analyticsChanged) {
        SimulationPipeline::AnalyticsParams updated = analyticsParams;
        updated.accelSaturation = std::max(0.0, static_cast<double>(accelSat));
        updated.gyroSaturation = std::max(0.0, static_cast<double>(gyroSat));
        updated.maxLagSamples = static_cast<std::size_t>(std::max(1, maxLag));
        updated.allanMaxTau = std::max(0.1, static_cast<double>(allanTau));
        updated.allanSteps = static_cast<std::size_t>(std::max(1, allanSteps));
        pipeline_.setAnalyticsParams(updated);
        analyticsStatus_.clear();
        monteCarloStatus_.clear();
    }

    ImGui::Separator();
    ImGui::Text("Monte Carlo");

    auto mcParams = pipeline_.monteCarloParams();
    int mcSamples = static_cast<int>(mcParams.samples);
    float mcAccelSigma = static_cast<float>(mcParams.accelBiasSigma);
    float mcGyroSigma = static_cast<float>(mcParams.gyroBiasSigma);
    float mcLeverSigma = static_cast<float>(mcParams.leverArmSigma);
    int mcSeed = static_cast<int>(mcParams.seed);

    bool mcChanged = false;

    if (ImGui::InputInt("Samples", &mcSamples)) {
        mcChanged = true;
    }
    if (ImGui::InputFloat("Accel Bias Sigma", &mcAccelSigma, 0.0f, 0.0f, "%.4f")) {
        mcChanged = true;
    }
    if (ImGui::InputFloat("Gyro Bias Sigma", &mcGyroSigma, 0.0f, 0.0f, "%.5f")) {
        mcChanged = true;
    }
    if (ImGui::InputFloat("Lever Arm Sigma", &mcLeverSigma, 0.0f, 0.0f, "%.4f")) {
        mcChanged = true;
    }
    if (ImGui::InputInt("Random Seed", &mcSeed)) {
        mcChanged = true;
    }

    if (mcChanged) {
        SimulationPipeline::MonteCarloParams updated = mcParams;
        updated.samples = static_cast<std::size_t>(std::max(0, mcSamples));
        updated.accelBiasSigma = std::max(0.0, static_cast<double>(mcAccelSigma));
        updated.gyroBiasSigma = std::max(0.0, static_cast<double>(mcGyroSigma));
        updated.leverArmSigma = std::max(0.0, static_cast<double>(mcLeverSigma));
        updated.seed = static_cast<unsigned int>(std::max(0, mcSeed));
        pipeline_.setMonteCarloParams(updated);
        monteCarloStatus_.clear();
    }

    if (ImGui::Button("Run Monte Carlo")) {
        if (pipeline_.runMonteCarlo()) {
            monteCarloStatus_ = "Monte Carlo envelope computed";
        } else {
            monteCarloStatus_ = "Monte Carlo run failed (check data/samples)";
        }
    }
    ImGui::SameLine();
    ImGui::Checkbox("Show samples", &showMonteCarloSamples_);

    if (!monteCarloStatus_.empty()) {
        ImGui::TextWrapped("%s", monteCarloStatus_.c_str());
    }

    ImGui::End();
}

void GuiApplication::drawAnalytics() {
    ImGui::Begin("Analytics");

    if (ImGui::Button("Compute Analytics")) {
        if (pipeline_.computeAnalytics()) {
            analyticsStatus_ = "Analytics computed";
        } else {
            analyticsStatus_ = "Not enough data for analytics";
        }
    }
    if (!analyticsStatus_.empty()) {
        ImGui::TextWrapped("%s", analyticsStatus_.c_str());
    }

    const auto& analytics = pipeline_.analytics();

    if (analytics.hasQuality) {
        ImGui::Separator();
        ImGui::Text("Quality Metrics");
        ImGui::Text("Samples: %zu", analytics.quality.sampleCount);
        ImGui::Text("Mean dt: %.6f s", analytics.quality.meanSamplingInterval);
        ImGui::Text("Nominal Rate: %.2f Hz", analytics.quality.nominalFrequency);
        ImGui::Text("Sampling Jitter: %.6f s", analytics.quality.samplingJitter);
        ImGui::Text("Missing Rate: %.2f%%", analytics.quality.missingDataRate * 100.0);
        ImGui::Text("Accel Saturation: %.2f%%", analytics.quality.saturationAccelRate * 100.0);
        ImGui::Text("Gyro Saturation: %.2f%%", analytics.quality.saturationGyroRate * 100.0);
    }

    if (analytics.hasTimeSync) {
        ImGui::Separator();
        ImGui::Text("Time Synchronization");
        ImGui::Text("Offset: %.4f s", analytics.timeSync.offset);
        ImGui::Text("Peak Correlation: %.3f", analytics.timeSync.peakCorrelation);
    }

    if (analytics.hasAllan) {
        ImGui::Separator();
        ImGui::Text("Allan Deviation");
        if (!analytics.accelAllan.tau.empty()) {
            ImGui::Text("Accelerometer (m/s^2)");
            ImGui::BeginTable("accallan", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
            ImGui::TableSetupColumn("tau [s]");
            ImGui::TableSetupColumn("sigma_x");
            ImGui::TableSetupColumn("sigma_y");
            ImGui::TableSetupColumn("sigma_z");
            ImGui::TableHeadersRow();
            for (std::size_t i = 0; i < analytics.accelAllan.tau.size(); ++i) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%.3f", analytics.accelAllan.tau[i]);
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.3e", analytics.accelAllan.sigmaX[i]);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.3e", analytics.accelAllan.sigmaY[i]);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.3e", analytics.accelAllan.sigmaZ[i]);
                if (i > 10) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("...");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("...");
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("...");
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("...");
                    break;
                }
            }
            ImGui::EndTable();
        }

        if (!analytics.gyroAllan.tau.empty()) {
            ImGui::Text("Gyroscope (rad/s)");
            ImGui::BeginTable("gyroad", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
            ImGui::TableSetupColumn("tau [s]");
            ImGui::TableSetupColumn("sigma_x");
            ImGui::TableSetupColumn("sigma_y");
            ImGui::TableSetupColumn("sigma_z");
            ImGui::TableHeadersRow();
            for (std::size_t i = 0; i < analytics.gyroAllan.tau.size(); ++i) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%.3f", analytics.gyroAllan.tau[i]);
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.3e", analytics.gyroAllan.sigmaX[i]);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.3e", analytics.gyroAllan.sigmaY[i]);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.3e", analytics.gyroAllan.sigmaZ[i]);
                if (i > 10) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("...");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("...");
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("...");
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("...");
                    break;
                }
            }
            ImGui::EndTable();
        }
    }

    const auto& mcEnv = pipeline_.monteCarloEnvelope();
    if (!monteCarloStatus_.empty()) {
        ImGui::Separator();
        ImGui::TextWrapped("Monte Carlo status: %s", monteCarloStatus_.c_str());
    }
    if (!mcEnv.empty()) {
        ImGui::Separator();
        ImGui::Text("Monte Carlo Results");
        ImGui::Text("Envelope points: %zu", mcEnv.size());
        ImGui::Text("Accepted samples: %zu", pipeline_.monteCarloSamples().size());
    }

    ImGui::End();
}

bool GuiApplication::exportSvg(const std::string& path) const {
    std::ofstream svg(path);
    if (!svg.is_open()) {
        return false;
    }

    const float width = 800.0f;
    const float height = 800.0f;
    const float margin = 80.0f;
    const float drawable = width - 2.0f * margin;
    const float minAxis = -axisRange_;
    const float maxAxis = axisRange_;
    const float axisSpan = maxAxis - minAxis;
    int ticks = std::max(2, tickCount_);
    float tickStep = axisSpan / static_cast<float>(ticks);

    auto mapPoint = [&](const Point2D& pt) {
        float xNorm = (static_cast<float>(pt.x) - minAxis) / axisSpan;
        float yNorm = (static_cast<float>(pt.y) - minAxis) / axisSpan;
        float x = margin + xNorm * drawable;
        float y = height - (margin + yNorm * drawable);
        return std::pair<float, float>{x, y};
    };

    svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width << "' height='" << height
        << "' viewBox='0 0 " << width << ' ' << height << "'>\n";
    svg << "  <rect x='0' y='0' width='" << width << "' height='" << height << "' fill='#1e1e23' />\n";

    float zeroX = mapPoint(Point2D{0.0, 0.0}).first;
    float zeroY = mapPoint(Point2D{0.0, 0.0}).second;
    svg << "  <line x1='" << zeroX << "' y1='" << margin << "' x2='" << zeroX << "' y2='" << (height - margin)
        << "' stroke='#b0b0c0' stroke-width='1.5' />\n";
    svg << "  <line x1='" << margin << "' y1='" << zeroY << "' x2='" << (width - margin) << "' y2='" << zeroY
        << "' stroke='#b0b0c0' stroke-width='1.5' />\n";

    svg << "  <text x='" << (width - margin + 20.0f) << "' y='" << (zeroY + 20.0f)
        << "' fill='#d0d0df' font-family='sans-serif' font-size='18'>Gx [g]</text>\n";
    svg << "  <text x='" << (zeroX + 20.0f) << "' y='" << (margin - 20.0f)
        << "' fill='#d0d0df' font-family='sans-serif' font-size='18'>Gy [g]</text>\n";

    for (int i = 0; i <= ticks; ++i) {
        float value = minAxis + i * tickStep;
        auto vx0 = mapPoint(Point2D{value, minAxis});
        auto vx1 = mapPoint(Point2D{value, maxAxis});
        svg << "  <line x1='" << vx0.first << "' y1='" << vx0.second << "' x2='" << vx1.first << "' y2='" << vx1.second
            << "' stroke='#3e3e46' stroke-width='1' />\n";

        auto vy0 = mapPoint(Point2D{minAxis, value});
        auto vy1 = mapPoint(Point2D{maxAxis, value});
        svg << "  <line x1='" << vy0.first << "' y1='" << vy0.second << "' x2='" << vy1.first << "' y2='" << vy1.second
            << "' stroke='#3e3e46' stroke-width='1' />\n";

        char label[32];
        std::snprintf(label, sizeof(label), "%0.2f", value);
        svg << "  <text x='" << (mapPoint(Point2D{value, 0.0}).first - 12.0f) << "' y='" << (zeroY + 18.0f)
            << "' fill='#c0c0cf' font-family='sans-serif' font-size='14'>" << label << "</text>\n";
        svg << "  <text x='" << (zeroX + 8.0f) << "' y='" << (mapPoint(Point2D{0.0, value}).second + 4.0f)
            << "' fill='#c0c0cf' font-family='sans-serif' font-size='14'>" << label << "</text>\n";
    }

    svg << "  <g fill='#a0a0a0' fill-opacity='0.6'>\n";
    for (const auto& point : pipeline_.processedPoints()) {
        auto [x, y] = mapPoint(point);
        svg << "    <circle cx='" << x << "' cy='" << y << "' r='2' />\n";
    }
    svg << "  </g>\n";

    svg << "  <g fill='#50a0ff' fill-opacity='0.85'>\n";
    for (const auto& point : pipeline_.steadyStatePoints()) {
        auto [x, y] = mapPoint(point);
        svg << "    <circle cx='" << x << "' cy='" << y << "' r='3' />\n";
    }
    svg << "  </g>\n";

    if (!pipeline_.monteCarloSamples().empty()) {
        svg << "  <g fill='#ffe080' fill-opacity='0.55'>\n";
        for (const auto& point : pipeline_.monteCarloSamples()) {
            auto [x, y] = mapPoint(point);
            svg << "    <circle cx='" << x << "' cy='" << y << "' r='1.6' />\n";
        }
        svg << "  </g>\n";
    }

    const auto& hull = pipeline_.envelopeHull();
    if (hull.size() >= 2) {
        svg << "  <polyline fill='none' stroke='#ff7474' stroke-width='2' points='";
        for (const auto& point : hull) {
            auto [x, y] = mapPoint(point);
            svg << x << ',' << y << ' ';
        }
        if (!hull.empty()) {
            auto [x, y] = mapPoint(hull.front());
            svg << x << ',' << y;
        }
        svg << "' />\n";
    }

    const auto& mcHull = pipeline_.monteCarloEnvelope();
    if (mcHull.size() >= 2) {
        svg << "  <polyline fill='none' stroke='#ffd200' stroke-width='2' points='";
        for (const auto& point : mcHull) {
            auto [x, y] = mapPoint(point);
            svg << x << ',' << y << ' ';
        }
        if (!mcHull.empty()) {
            auto [x, y] = mapPoint(mcHull.front());
            svg << x << ',' << y;
        }
        svg << "' />\n";
    }

    svg << "</svg>\n";
    return true;
}

}  // namespace gg::gui
