#include "test_scene.h"


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