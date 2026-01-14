#include "Labs/Final_Project/App.h"
#include "Assets/bundled.h"

namespace VCX::Labs::Rendering {
    using namespace Assets;

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _caseRayTracing({ ExampleScene::Floor, ExampleScene::CornellBox, ExampleScene::WhiteOak, ExampleScene::SportsCar, ExampleScene::BreakfastRoom, ExampleScene::Sibenik, ExampleScene::Sponza }),
        _casePathTracing({ ExampleScene::Floor, ExampleScene::CornellBox, ExampleScene::WhiteOak, ExampleScene::SportsCar, ExampleScene::BreakfastRoom, ExampleScene::Sibenik, ExampleScene::Sponza }) {}

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
