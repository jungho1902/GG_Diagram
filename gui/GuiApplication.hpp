#pragma once

#include "SimulationPipeline.hpp"

#include <memory>
#include <string>

struct GLFWwindow;

namespace gg::gui {

class GuiApplication {
public:
    GuiApplication();
    ~GuiApplication();

    GuiApplication(const GuiApplication&) = delete;
    GuiApplication& operator=(const GuiApplication&) = delete;

    void run();

private:
    void initWindow();
    void initImGui();
    void shutdown();

    void drawControls();
    void drawPlot();
    void drawParameters();
    void drawAnalytics();
    bool exportSvg(const std::string& path) const;

    GLFWwindow* window_{nullptr};
    SimulationPipeline pipeline_;
    bool shouldClose_{false};

    const float axisRange_{2.0f};
    int tickCount_{8};
    std::string exportPath_{"gg_diagram.svg"};
    std::string exportStatus_;
    bool useDataset_{false};
    std::string datasetPath_;
    std::string datasetStatus_;
    std::string resultsDir_;
    std::string resultsStatus_;
    float datasetPlayback_{1.0f};
    std::string analyticsStatus_;
    bool showMonteCarloSamples_{false};
    std::string monteCarloStatus_;
};

}  // namespace gg::gui
