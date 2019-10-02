#ifndef SMEAR_H
#define SMEAR_H	

#include <map>

#include <maya/MPxDeformerNode.h>
#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>

#include <maya/MArrayDataHandle.h>
#include <maya/MDataHandle.h>
#include <maya/MPlug.h> 
#include <maya/MPoint.h> 
#include <maya/MPointArray.h> 
#include <maya/MMatrix.h> 
#include <maya/MTypeId.h> 
#include <maya/MFloatVectorArray.h>
#include <maya/MVector.h>
#include <maya/MGlobal.h>
#include <maya/MTime.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MRampAttribute.h>


class mSmear : public MPxDeformerNode {
public:
	mSmear();
	~mSmear();
	
	static void* creator();
	static MStatus initialize();
	
	void postConstructor();

	MStatus postConstructor_init_curveRamp(MObject& nodeObj, MObject& rampObj, int index, float position, 
		float value,int interpolation);

	MStatus deform(MDataBlock& data, MItGeometry& iter,	const MMatrix& matrix,	unsigned int multiIndex);
		
	static MTypeId id;
	static MObject aTime;
	static MObject aReverse;
	static MObject aSmearStrength;
	static MObject aStartFrame;
	static MObject curveRamp;

	// Store everything per input geometry
	std::map<unsigned int, MTime> previousTime_;
	std::map<unsigned int, bool> initialized_;
	std::map<unsigned int, MPointArray> previousPoints_;
	std::map<unsigned int, MPointArray> currentPoints_;
	
};

#endif
