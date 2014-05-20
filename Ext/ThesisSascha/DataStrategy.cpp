/*
 * DataStrategy.cpp
 *
 *  Created on: 20.05.2014
 *      Author: MeisterYeti
 */
#ifdef MINSG_EXT_THESISSASCHA

#include "DataStrategy.h"
#include "Definitions.h"
#include "SurfelManager.h"

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/Mesh/MeshIndexData.h>
#include <Rendering/Mesh/MeshVertexData.h>
#include <Rendering/RenderingContext/RenderingContext.h>

namespace MinSG {
namespace ThesisSascha {
using namespace Rendering;


void DataStrategy::assureLocalVertexData(Rendering::Mesh * mesh) {
	MeshVertexData & vd=mesh->_getVertexData();

	if( vd.dataSize()==0 && vd.isUploaded())
		vd.download();
}

void DataStrategy::assureLocalIndexData(Rendering::Mesh * mesh) {
	MeshIndexData & id=mesh->_getIndexData();

	if( id.dataSize()==0 && id.isUploaded())
		id.download();
}

void DataStrategy::displayMesh(Rendering::RenderingContext & context, Rendering::Mesh * m, uint32_t startIndex, uint32_t indexCount) {
	if(!m->empty())
		DataStrategy::doDisplayMesh(context, m, startIndex, indexCount);
}

}
}

#endif /* MINSG_EXT_THESISSASCHA */
