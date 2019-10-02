/*
		eSmear - Deformer--v001

	Eder DG Personal Project(2019)
*/

#include "edBlur.h"

//------------------------------------------------
//Static Variables 
//------------------------------------------------
MTypeId mSmear::id(0x0ED100893);
MObject mSmear::aTime;
MObject mSmear::aStartFrame;
MObject mSmear::aReverse;
MObject mSmear::aSmearStrength;
MObject mSmear::curveRamp;

//------------------------------------------------
//Constructor & Deconstructor
//------------------------------------------------
mSmear::mSmear()
{}

mSmear::~mSmear()
{}

void* mSmear::creator()
{
	return new mSmear();
}


//------------------------------------------------
//initialize the attrs
//------------------------------------------------
MStatus mSmear::initialize() {
	
	MStatus status;
	
	//MFnNumericAttribute

	MFnNumericAttribute nAttr;

	aSmearStrength = nAttr.create("smearStrength", "smearStrength", MFnNumericData::kFloat);
	nAttr.setMin(0);
	nAttr.setMax(1.0);
	nAttr.setDefault(0.5);
	nAttr.setKeyable(true);
		
	aReverse = nAttr.create("reverse", "reverse", MFnNumericData::kBoolean, 0);
	nAttr.setKeyable(true);

	aStartFrame = nAttr.create("startFrame", "startFrame", MFnNumericData::kInt, 0);
	nAttr.setKeyable(true);
	
	// MRampAttribute

	MRampAttribute rAttr;

	curveRamp = rAttr.createCurveRamp("bulgeShape", "bulgeShape");

	// MFnUnitAttribute

	MFnUnitAttribute uAttr;
	
	aTime = uAttr.create("time", "time", MFnUnitAttribute::kTime, 0.0);
	
	//addAttribute
	addAttribute(aSmearStrength);
	addAttribute(aReverse);
	addAttribute(aStartFrame);
	addAttribute(curveRamp);
	addAttribute(aTime);
	
	//attributeAffects
	attributeAffects(aTime, outputGeom);
	attributeAffects(aSmearStrength, outputGeom);
	attributeAffects(aReverse, outputGeom);
	attributeAffects(aStartFrame, outputGeom);
	attributeAffects(curveRamp, outputGeom);
	
	return MS::kSuccess;
}

// ---------------------------------------------------------------------
// initialize the ramp attribute
// ---------------------------------------------------------------------

void mSmear::postConstructor()
{
	MStatus status;
	MObject thisNode = this->thisMObject();

	// One entry is the least needed or the attribute editor will
	// produce an error.

	postConstructor_init_curveRamp(thisNode, curveRamp, 0, 0.0f, 0.0f, 3);
	postConstructor_init_curveRamp(thisNode, curveRamp, 1, 0.2f, 1.0f, 2);
	postConstructor_init_curveRamp(thisNode, curveRamp, 2, 1.0f, 0.0f, 2);

}

MStatus mSmear::postConstructor_init_curveRamp(MObject& nodeObj, MObject& rampObj, int index, float position,
	float value, int interpolation)
{
	MStatus status;

	MPlug rampPlug(nodeObj, rampObj);
	
	MPlug elementPlug = rampPlug.elementByLogicalIndex((unsigned)index, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug positionPlug = elementPlug.child(0);
	positionPlug.setFloat(position);
	MPlug valuePlug = elementPlug.child(1);
	valuePlug.setFloat(value);
	MPlug interPlug = elementPlug.child(2);
	interPlug.setInt(interpolation);

	return MS::kSuccess;
}

//------------------------------------------------
//Deformer Function
//------------------------------------------------

MStatus mSmear::deform(MDataBlock& data,MItGeometry& iter,const MMatrix& matrix,unsigned int multiIndex) 
{
	MStatus status;

	MObject thisNode = this->thisMObject();

	float smearStrength = data.inputValue(aSmearStrength).asFloat();
	bool reverse = data.inputValue(aReverse).asBool();
	int startFrame = data.inputValue(aStartFrame).asInt();

	MRampAttribute curveAttr = MRampAttribute(thisNode, curveRamp, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	//getting the current time
	MTime currentTime = data.inputValue(aTime).asTime();
	
	//getting the deformer envelope
	float env = data.inputValue(envelope).asFloat();
	
	if (env == 0)
		return MS::kSuccess;

	// -----------------------------------------------------------------
	// store the mesh points
	// -----------------------------------------------------------------

	MPointArray points;
	iter.allPositions(points);

	// -----------------------------------------------------------------
	// get the input mesh
	// -----------------------------------------------------------------

	MArrayDataHandle inputArrayHandle = data.outputArrayValue(input, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = inputArrayHandle.jumpToArrayElement(multiIndex);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// check if the input geometry is a mesh
	MDataHandle inputGeomHandle = inputArrayHandle.outputValue().child(inputGeom);

	unsigned vtxCount = (unsigned)iter.count();

	//get the normals 
	MObject meshObj = inputGeomHandle.asMesh();
	MFnMesh meshFn(meshObj);
	MFloatVectorArray normals;
	meshFn.getVertexNormals(false, normals);
	
	// -----------------------------------------------------------------
	// initalize the deormation
	// -----------------------------------------------------------------

	MPointArray& currentPoints = currentPoints_[multiIndex];
	MPointArray& previousPoints = previousPoints_[multiIndex];
	MTime& previousTime = previousTime_[multiIndex];
	if (!initialized_[multiIndex]) {
		previousTime = currentTime;
		initialized_[multiIndex] = true;
		currentPoints.setLength(iter.count());
		previousPoints.setLength(iter.count());
		for (unsigned int i = 0; i < points.length(); ++i) {
			currentPoints[i] = points[i]*matrix;
			previousPoints[i] = currentPoints[i];
		}
	}

	// Check if the timestep is just 1 frame 
	double timeDifference = currentTime.value() - previousTime.value();
	if (timeDifference > 1.0 || timeDifference < 0.0 || currentTime.value() <= startFrame) {
		initialized_[multiIndex] = false;
		previousTime = currentTime;
		return MS::kSuccess;
	}
	
	
	MPoint currentPos, newPos;
	MVector velocity,displacement;
	double normalDot;
	float value,pos,distance;

	for (int i = 0; i < vtxCount; ++i) {
		// Track the position
		currentPos = points[i] * matrix;
		
		//setting the position
		newPos = currentPos;
		
		//calculate the displacement
		displacement = currentPos - currentPoints[i];

		//get the dotProduct
		normalDot = displacement.normal() * normals[i];

		if (normalDot < 0) {
			displacement *= 0.5;
			newPos += (displacement*normalDot)*smearStrength;
		}

		// Store the previous points
		previousPoints[i] = currentPoints[i];
		currentPoints[i] = newPos;

		// Multiply by weight map and envelope
		points[i] = newPos * matrix.inverse() * env;

	}

	iter.setAllPositions(points);
	previousTime = currentTime;

	return status;
}



