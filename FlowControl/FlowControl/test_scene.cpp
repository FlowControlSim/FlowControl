#include "test_scene.h"

MTypeId testScene::id(0x8000f);

void* testScene::creator()
{
	return new testScene();
}

MStatus testScene::initialize()
{
	MStatus status;
	return status;
}

MStatus testScene::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus status;
	return status;
}