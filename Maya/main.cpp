#include <maya/MStatus.h>
#include <maya/MObject.h>
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MSceneMessage.h>

#include <greenDeformer.h>

MStatus initializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin plugin(obj, "CTD", "1.0");

    //MSceneMessage::addCallback(MSceneMessage::kMayaExiting, somethingtoremoveyourbutton);

    // register node
    status = plugin.registerNode("greenDeformer",
                                 greenDeformer::id,
                                 greenDeformer::creator,
                                 greenDeformer::initialize);

    if (!status)
    {
        status.perror("registerNode(\"greenDeformer\", ...\)");
        return status;
    }

    // create button
    //int result;    
    //MGlobal::executeCommand("...", result);
    
    return status;
}


MStatus uninitializePlugin(MObject obj) 
{
    MStatus   status;
    MFnPlugin plugin(obj);

    // unregister node
    status =  plugin.deregisterNode(greenDeformer::id);
    if (!status)
    {
        status.perror("deregisterNode(greenDeformer::id, ...\)");
        return status;
    }

    // delete button and callback

    return status;
}
