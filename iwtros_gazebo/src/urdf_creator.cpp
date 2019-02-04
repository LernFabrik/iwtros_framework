#include "iwtros_gazebo/urdf_creator.h"
#include <ros/console.h>

namespace iwtros{
    namespace simulation{
        std::string createObjectURDF(const std::string& object_name, const std::string& mesh_filename){
            // Set up xml document
            TiXmlDocument doc;
            // Set up the xml decalaration line
            TiXmlDeclaration* decl (new TiXmlDeclaration("1.0", "", ""));

            //create URDF Tags
            GeometryTag visual_geometry (mesh_filename);
            VisualTag visual (visual_geometry);
            GeometryTag collision_geometry (mesh_filename);
            CollisionTag collision (collision_geometry);
            RobotTag robot (object_name, visual, collision);

            //set up hierarchy
            doc.LinkEndChild(decl);
            doc.LinkEndChild(robot.self);

            // Set up printer 
            TiXmlPrinter printer;
            printer.SetIndent("  ");
            doc.Accept(&printer);
            std::string xmltext = printer.CStr();

            return xmltext;
        }
    }
}

