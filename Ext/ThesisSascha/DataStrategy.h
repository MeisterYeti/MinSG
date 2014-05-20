/*
 * DataStrategy.h
 *
 *  Created on: 20.05.2014
 *      Author: MeisterYeti
 */
#ifdef MINSG_EXT_THESISSASCHA

#ifndef DATASTRATEGY_H_
#define DATASTRATEGY_H_

#include <Rendering/Mesh/MeshDataStrategy.h>
#include <Util/References.h>

namespace Rendering {
class Mesh;
class RenderingContext;
}

namespace MinSG {
namespace ThesisSascha {
class SurfelManager;

class DataStrategy : public Rendering::MeshDataStrategy {
public:
	DataStrategy(SurfelManager* manager) : Rendering::MeshDataStrategy(), manager(manager) {}
	virtual ~DataStrategy() {}

	void assureLocalVertexData(Rendering::Mesh * m) override;

	void assureLocalIndexData(Rendering::Mesh * m) override;

	void prepare(Rendering::Mesh * /*mesh*/) override {
	}

	void displayMesh(Rendering::RenderingContext & context, Rendering::Mesh * m, uint32_t startIndex, uint32_t indexCount) override;

private:
	Util::Reference<SurfelManager> manager;
};

}
}
#endif /* DATASTRATEGY_H_ */
#endif /* MINSG_EXT_THESISSASCHA */
