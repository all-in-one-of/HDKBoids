/*
Author: Soumitra Goswami
Description: Generates Flocking behavior based on the paper by Craig Reynolds (https://www.red3d.com/cwr/papers/1987/boids.html). 
             This plugin is using the Houdini Dev Kit (HDK 2014), written in C++.
Date: 09/15/2015
Latest Edit: 02/20/2018 - Commenting out the file
*/

#ifndef __SGParticleGen_h__
#define __SGParticleGen_h__

//SOP Context
#include <SOP/SOP_Node.h>

#define INT_PARM(name, idx, vidx, t)	\
	    return evalInt(name, &myOffsets[idx], vidx, t);

#define FLT_PARM(name, idx, vidx, t)	\
	    return evalFloat(name, &myOffsets[idx], vidx, t);

class GEO_ParticleVertex;
class GEO_PrimParticle;
class GU_RayIntersect;

namespace hdk_test{
	
	class SGParticleGen: public SOP_Node //Specifies the node is in the SOP Context in Houdini
	{
	public:
		SGParticleGen(OP_Network *net, const char *name, OP_Operator *op);
		virtual ~SGParticleGen();

		static PRM_Template	myTemplateList[];//Contains the name of parameters to expose in Houdini
		static OP_Node		*myConstructor(OP_Network*, const char *,OP_Operator *);

	protected:
	
		//Emits and sets the attributes for a single particle from either the origin or geometry provided.
		void Emit(int id);
		//Simple Euler Integration of particles based on the provided force. Updates velocity and position.
		int Move(GA_Offset ptoff, const UT_Vector3 &force);
		
		//Initializes the Particle System and it's initial attributes.
		void initSystem();
		
		//Update function.
		void timeStep(fpreal now);
		
		//Calculates all the interparticulate forces acting on the particle
		UT_Vector3 localForces(GA_Size j, UT_Vector3 force, fpreal now); 
		
		//Accumulates all the different (global, local, steer) forces acting on the flocking particles
		UT_Vector3 calcForces(GA_Offset ptoff, GA_Size j, fpreal now);
		
		//Steer force that keeps the particles in a flock.
		UT_Vector3 steer(GA_Offset ptoff,UT_Vector3 force,fpreal now);
		
		// Method to cook geometry for the SOP
		virtual OP_ERROR		 cookMySop(OP_Context &context);
		
		//Sets Label name for the inputs
		virtual const char *inputLabel(unsigned idx) const;

	private:
		
		void ResetSystem(); // resets the system
		//void AddParticleAttrs();
		//Parameters to be evaluated
		int STARTFRAME()			{INT_PARM("startFrame",0,0,0)}
		int MAXNUM()				{INT_PARM("maxNum",3,0,0)}
		int BIRTH(fpreal t)			{INT_PARM("birth",11,0,t)};
		fpreal32 ORIGINX()			{FLT_PARM("center",1,0,0)}
		fpreal32 ORIGINY()			{FLT_PARM("center",1,1,0)}
		fpreal32 ORIGINZ()			{FLT_PARM("center",1,2,0)}
		fpreal32 INITVELX()			{FLT_PARM("initVel",32,0,0)}
		fpreal32 INITVELY()			{FLT_PARM("initVel",32,1,0)}
		fpreal32 INITVELZ()			{FLT_PARM("initVel",32,2,0)}
		fpreal LIFEMIN()			{FLT_PARM("lifeSpanMin",9,0,0)}
		fpreal LIFEMAX()			{FLT_PARM("lifeSpanMax",10,0,0)}
		fpreal32 VARX()				{FLT_PARM("variance",12,0,0)}
		fpreal32 VARY()				{FLT_PARM("variance",12,1,0)}
		fpreal32 VARZ()				{FLT_PARM("variance",12,2,0)}
		fpreal AIRRES()				{FLT_PARM("airRes", 14,0,0)}
		fpreal32 WINDX()			{FLT_PARM("windForce",15,0,0)}
		fpreal32 WINDY()			{FLT_PARM("windForce",15,1,0)}
		fpreal32 WINDZ()			{FLT_PARM("windForce",15,2,0)}
		fpreal32 GRAVX()			{FLT_PARM("gravity",16, 0, 0)}
		fpreal32 GRAVY()			{FLT_PARM("gravity",16, 1, 0)}
		fpreal32 GRAVZ()			{FLT_PARM("gravity",16, 2, 0)}
		fpreal MASS()				{FLT_PARM("mass", 17, 0, 0)}
		fpreal KA()				{FLT_PARM("ka",   18, 0, 0)}
		fpreal KV()				{FLT_PARM("kv",   19, 0, 0)}
		fpreal KC()				{FLT_PARM("ka",	20,0,0)}
		fpreal KG()				{FLT_PARM("kG", 24, 0, 0)}
		fpreal TC()				{FLT_PARM("tC",31,0,0)}
		fpreal INFRAD()				{FLT_PARM("infRad", 21, 0 ,0)}
		fpreal BSIZE()				{FLT_PARM("bSize",25,0,0)}
		fpreal COLRAD()				{FLT_PARM("colRad",30,0,0)}
		fpreal MAXACC()				{FLT_PARM("maxAcc", 22, 0 ,0)}
		fpreal32 GOALX()			{FLT_PARM("goal", 23, 0, 0)}
		fpreal32 GOALY()			{FLT_PARM("goal", 23, 1, 0)}
		fpreal32 GOALZ()			{FLT_PARM("goal", 23, 2, 0)}
		fpreal FANGLE()				{FLT_PARM("fAngle",26,0,0)}
		fpreal PANGLE()				{FLT_PARM("pAngle",27,0,0)}
		fpreal LEADINF()			{FLT_PARM("leadInf",28,0,0)}
		fpreal32 COLPOSX(fpreal t)		{FLT_PARM("colPos",29,0,t)}
		fpreal32 COLPOSY(fpreal t)		{FLT_PARM("colPos",29,1,t)}
		fpreal32 COLPOSZ(fpreal t)		{FLT_PARM("colPos",29,2,t)}
		
		

		std::list<part*> my_partList;
		std::list<part*>::iterator my_part_it;
		
		//variables to store Source node data
		const GU_Detail *mySource; 
		GA_Index	mySourceNum;  
		GA_ROHandleV3	mySourceVel;

		GEO_PrimParticle *my_gpp; //keeps all the particle information
		
		fpreal my_prevTime; 
		fpreal my_timeDiff; 
		fpreal my_lastCookTime; //tracks last frame that was cooked
		fpreal my_drag; //drag force
		GA_RWHandleV3	myVelocity; //Velocity
		GA_RWHandleV3	myvariance;//velocity variance
		GA_RWHandleF	myLife; //Life
		GA_RWHandleF	myage; //Age
		GA_RWHandleV3	mySteer; //Steer force
		GA_RWHandleV3	mylforce; //force
		GA_RWHandleI my_global_id; //id
		
		static int *myOffsets;
	};


}
#endif //SGParticleGen.h
