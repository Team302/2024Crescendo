#pragma once

#include <string>
#include "DragonVision/DragonVisionStructs.h"
#include "DragonVision/DragonCamera.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Transform3d.h"

class DragonVisionUtils
{
public:
    static bool CompareVisionData(std::optional<VisionData> data1, std::optional<VisionData> data2);
};