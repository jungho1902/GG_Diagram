#include "gui/GuiApplication.hpp"

#include <cstdlib>
#include <exception>
#include <iostream>

int main() {
    try {
        gg::gui::GuiApplication app;
        app.run();
        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Fatal error: " << ex.what() << '\n';
    } catch (...) {
        std::cerr << "Fatal error: unknown exception" << '\n';
    }
    return EXIT_FAILURE;
}

