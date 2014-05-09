#ifdef MINSG_EXT_THESISSASCHA

#ifndef THESISSASCHA_IMPORTHANDLER_H_
#define THESISSASCHA_IMPORTHANDLER_H_

#include <MinSG/SceneManagement/Importer/MeshImportHandler.h>
#include <Util/References.h>

namespace Util {
class FileName;
}
namespace MinSG {
class Node;
namespace ThesisSascha {
class SurfelManager;

/**
 * Class that registers itself at the SceneManager and then takes notice whenever a new mesh is to be loaded.
 *
 * @author Sascha Brandt
 */
class ImportHandler : public SceneManagement::MeshImportHandler {
	public:
		ImportHandler(SurfelManager* manager) : manager(manager) {};
		virtual ~ImportHandler() {
		}

		/**
		 * Ignores mesh data and returns an empty GeometryNode.
		 *
		 * @param url Location of the mesh file.
		 * @param description Description of the Node to which the mesh belongs.
		 * @return Arbitrary node or tree of nodes that represents the mesh inside the scene graph.
		 */
		 Node * handleImport(const Util::FileLocator& locator, const std::string & url, const SceneManagement::NodeDescription * description) override;
		 Node * handleImport(const SceneManagement::NodeDescription * description) override;

		 static void pushImportHandler(SurfelManager* manager_);

		 static void popImportHandler();
	private:
		 Util::Reference<SurfelManager> manager;
};

}
}

#endif /* THESISSASCHA_IMPORTHANDLER_H_ */

#endif /* MINSG_EXT_THESISSASCHA */
