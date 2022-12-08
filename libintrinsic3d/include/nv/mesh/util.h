#pragma once

#include <nv/mat.h>
#include <nv/mesh.h>


namespace nv
{
    namespace MeshUtil
    {
        void removeLooseComponents(Mesh* mesh);
        void removeUnusedVertices(Mesh* mesh);
        bool removeDegenerateFaces(Mesh* mesh);
    } 
} 
