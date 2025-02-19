// Copyright (c) 2010, Willow Garage, Inc.
// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "rviz_rendering/mesh_loader.hpp"

#include <resource_retriever/memory_resource.hpp>
#include <resource_retriever/plugins/retriever_plugin.hpp>
#include <string>

#include "OgreHardwareBufferManager.h"
#include "OgreMaterial.h"
#include "OgreMaterialManager.h"
#include "OgreMeshManager.h"
#include "OgreMeshSerializer.h"
#include "OgrePass.h"
#include "OgreSubMesh.h"
#include "OgreTechnique.h"
#include "OgreTextureManager.h"
#include "OgreVector.h"

#include <QDir>  // NOLINT cpplint cannot handle include order here
#include <QFileInfo>  // NOLINT cpplint cannot handle include order here
#include <QString>  // NOLINT cpplint cannot handle include order here

#define ASSIMP_UNIFIED_HEADER_NAMES 1
#if defined(ASSIMP_UNIFIED_HEADER_NAMES)
#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"
#include "assimp/IOStream.hpp"
#include "assimp/IOSystem.hpp"
#else
#include "assimp/assimp.hpp"
#include "assimp/aiScene.h"
#include "assimp/aiPostProcess.h"
#include "assimp/IOStream.h"
#include "assimp/IOSystem.h"
#endif

#include "resource_retriever/retriever.hpp"

#include "mesh_loader_helpers/assimp_loader.hpp"
#include "rviz_rendering/logging.hpp"

namespace rviz_rendering
{

Ogre::MeshPtr loadMeshFromResource(
  resource_retriever::Retriever * retriever,
  const std::string & resource_uri)
{
  // Early exit with empty retriever
  if (nullptr == retriever) {
    RVIZ_RENDERING_LOG_ERROR("retriever is unexpectedly nullptr");
    return nullptr;
  }

  // Check for cached resources
  if (Ogre::MeshManager::getSingleton().resourceExists(resource_uri, ROS_PACKAGE_NAME)) {
    return Ogre::MeshManager::getSingleton().getByName(resource_uri, ROS_PACKAGE_NAME);
  }

  // Check for ".mesh" resources, which OGRE can load directly
  QFileInfo model_path(QString::fromStdString(resource_uri));
  std::string ext = model_path.completeSuffix().toStdString();
  if (ext == "mesh" || ext == "MESH") {
    auto res = retriever->get_shared(resource_uri);
    if (res == nullptr || res->data.empty()) {
      return nullptr;
    }
    Ogre::MeshSerializer ser;
    Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(*res->data.data(), res->data.size()));
    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(resource_uri, ROS_PACKAGE_NAME);
    ser.importMesh(stream, mesh.get());
    stream->close();
    return mesh;
  }

  {
    AssimpLoader assimp_loader(retriever);
    const aiScene * scene = assimp_loader.getScene(resource_uri);
    if (scene == nullptr) {
      RVIZ_RENDERING_LOG_ERROR_STREAM(
        "Could not load resource [" << resource_uri.c_str() << "]: " <<
          assimp_loader.getErrorMessage());
      return nullptr;
    }
    return assimp_loader.meshFromAssimpScene(resource_uri, scene);
  }
}

}  // namespace rviz_rendering
