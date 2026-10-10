#include "app/App.h"

#include <cstdio>

int main()
{
    App app;
    if (!app.init(1600, 1000, "huge-atlas")) {
        std::fprintf(stderr, "Initialization failed\n");
        app.shutdown();
        return 1;
    }
    app.run();
    app.shutdown();
    return 0;
}
