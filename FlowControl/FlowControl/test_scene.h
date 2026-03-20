#ifndef TEST_SCENE_H
#define TEST_SCENE_H

#include <maya/MPxNode.h>
#include <maya/MGlobal.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MTypeId.h>

MTypeId testScene::id(0x8000f);

class testScene : public MPxNode {

public: 
	testScene() {};
	virtual ~testScene();
	static void* creator();
	static MStatus initialize();
	static MStatus compute(const MPlug& plug, MDataBlock& data);
	static MTypeId id;

};

#endif // TEST_SCENE_H