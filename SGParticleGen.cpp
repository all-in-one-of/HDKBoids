#include <GU/GU_Detail.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>

#include <SOP/SOP_Guide.h>

#include <SYS/SYS_Types.h>
#include <PRM/PRM_Include.h>

#include <UT/UT_DSOVersion.h>
#include <GEO/GEO_PrimPart.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_Vector4.h>

#include <iostream>
#include <list>
#include "glm/glm.hpp"

#include "part.h"
#include "SGParticleGen.h"

#define INT_PARM(name, idx, vidx, t)	\
	    return evalInt(name, &myOffsets[idx], vidx, t);

#define FLT_PARM(name, idx, vidx, t)	\
	    return evalFloat(name, &myOffsets[idx], vidx, t);

using namespace hdk_test;
using namespace std;

float ptclLifeSpan = 30;
void newSopOperator(OP_OperatorTable *table)
{
	CH_LocalVariable* localVars = NULL;
	table->addOperator(new OP_Operator(
						"SG_fSys",
						"SG Simple Flocking",
						SGParticleGen::myConstructor,
						SGParticleGen::myTemplateList,
						0,
						2,
						localVars,
						0));
	 

}
//Parameter names
static PRM_Name		names[] = {
	PRM_Name("startFrame", "Start Frame"),			//0
	PRM_Name("center", "Center"),					//1
	PRM_Name("doCollision","Collision"),			//2
	PRM_Name("maxNum","Max Particles"),				//3
	
	PRM_Name("randVelMin", "Random Velocity Min"),	//4
	PRM_Name("randVelMax", "Random Velocity Max"),	//5
	PRM_Name("seed","Random Seed"),					//6

	PRM_Name("bounce", "Bounce"),					//7
	PRM_Name("bounceLimit", "Max Bounces"),			//8

	PRM_Name("lifeSpanMin", "LifeSpan Min"),		//9
	PRM_Name("lifeSpanMax", "LifeSpan Max"),		//10
	PRM_Name("birth", "Birth Rate"),				//11
	PRM_Name("variance","Velocity Variance"),		//12

	PRM_Name("airRes", "Air Resistance"),			//14
	PRM_Name("windForce", "Wind Force"),			//15
	PRM_Name("gravity", "Gravity"),					//16
	PRM_Name("mass", "Mass"),						//17

	PRM_Name("ka", "Avoidance Constant"),			//18
	PRM_Name("kv", "Vel Matching Constant"),		//19
	PRM_Name("kc", "Centering Constant"),			//20

	PRM_Name("infRad", "Influence Radius"),			//21
	PRM_Name("maxAcc", "Max Acceleration"),			//22

	PRM_Name("goal", "Goal Pos"),					//23
	PRM_Name("kG", "Goal Weight"),					//24
	PRM_Name("bSize","Boid Size"),					//25
	PRM_Name("fAngle"," Max Frontal Angle"),		//26
	PRM_Name("pAngle","Max Periferal Angle"),		//27
	PRM_Name("leadInf","Leader Influence"),			//28

	PRM_Name("colPos", "Collision Position"),		//29
	PRM_Name("colRad", "Collision Radius"),			//30
	PRM_Name("tC", "Threshold Collision time"),		//31		//in frames
	PRM_Name("initVel", "Initial Velocity")			//32		
};

static PRM_Range ranges[] = {
	PRM_Range(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1000000),
	PRM_Range(PRM_RANGE_UI, 0, PRM_RANGE_UI, 100),

};

static PRM_Default defaults[] = {
	PRM_Default(2000),			//0
	PRM_Default(3),				//1
	PRM_Default(100),			//2
	PRM_Default(1000),			//3	
	PRM_Default(-0.06674),		//4
	PRM_Default(10),			//5
	PRM_Default(0.1),			//6
	PRM_Default(60),			//7
	PRM_Default(120),			//8
};

static PRM_Default g1[] =
{
		PRM_Default(0),
		PRM_Default(-9.8),
		PRM_Default(0)
};


//Setting UI Parameters
PRM_Template SGParticleGen::myTemplateList[] = {
	PRM_Template(PRM_INT_J, 1, &names[0], PRMoneDefaults),
	PRM_Template(PRM_XYZ_J, 3, &names[1]),
	PRM_Template(PRM_INT_J, 1, &names[3], &defaults[0],0,&ranges[0]),
	PRM_Template(PRM_INT_J, 1, &names[6], &defaults[1], 0, &ranges[0]),
	PRM_Template(PRM_FLT_J, 1, &names[9], PRMoneDefaults),
	PRM_Template(PRM_FLT_J, 1, &names[10], &defaults[3]),
	PRM_Template(PRM_INT_J, 1, &names[11], &defaults[2]),
	PRM_Template(PRM_XYZ_J, 3, &names[12]),

	PRM_Template(PRM_FLT_J, 1, &names[13], PRMzeroDefaults),
	PRM_Template(PRM_XYZ_J, 3, &names[14]),
	PRM_Template(PRM_XYZ_J, 3, &names[15]),
	PRM_Template(PRM_FLT_J, 1, &names[16],PRMoneDefaults),
	PRM_Template(PRM_FLT_J, 1, &names[17],PRMzeroDefaults),
	PRM_Template(PRM_FLT_J, 1, &names[18],PRMzeroDefaults),
	PRM_Template(PRM_FLT_J, 1, &names[19], PRMzeroDefaults),
	PRM_Template(PRM_FLT_J, 1, &names[20], &defaults[1]),
	PRM_Template(PRM_FLT_J, 1, &names[21], &defaults[5]),
	PRM_Template(PRM_XYZ_J, 3, &names[22]),
	PRM_Template(PRM_FLT_J, 1, &names[23], PRMzeroDefaults),
	PRM_Template(PRM_FLT_J, 1, &names[24], PRMoneDefaults),
	PRM_Template(PRM_FLT_J, 1, &names[25], &defaults[7]),
	PRM_Template(PRM_FLT_J, 1, &names[26], &defaults[8]),
	PRM_Template(PRM_FLT_J, 1, &names[27], PRMoneDefaults),
	PRM_Template(PRM_XYZ_J, 3, &names[28]),
	PRM_Template(PRM_FLT_J, 1, &names[29], PRMoneDefaults),
	PRM_Template(PRM_FLT_J, 1, &names[30], &defaults[5]),
	PRM_Template(PRM_XYZ_J, 3, &names[31]),
	PRM_Template()
};

int * 
SGParticleGen::myOffsets = 0;

OP_Node* SGParticleGen::myConstructor(OP_Network *net, const char* name, OP_Operator *op)
{
	SGParticleGen *my_pGenerator = new SGParticleGen(net, name, op);
	my_pGenerator->my_global_id = 0;
	return my_pGenerator;

}

SGParticleGen::SGParticleGen(OP_Network *net, const char* name, OP_Operator *entry)
	: SOP_Node(net,name,entry),my_gpp(NULL)
{
	if(!myOffsets)
		myOffsets = allocIndirect(50);//Just to create array Allocating max 32 inputs

	myVelocity.clear();
}

SGParticleGen::~SGParticleGen()
{
}

void 
SGParticleGen::Emit(int id)
{
	GA_Offset srcptoff = GA_INVALID_OFFSET;
	GA_Offset vtxoff = my_gpp->giveBirth();//births a particle at origin

	//generating random numbers
	fpreal32 myVarx = (fpreal32)(SYSdrand48() * VARX()*2) - 1;
	fpreal32 myVary = (fpreal32)(SYSdrand48() * VARY()*2) - 1;
	fpreal32 myVarz = (fpreal32)(SYSdrand48() * VARZ()*2) - 1;
	
	
	if(mySource)//checks if I connected a source Geometry
	{
		if(mySourceNum >= mySource->getPointMap().indexSize())//If there are no points in the index
			mySourceNum = 0;
		if(mySource->getPointMap().indexSize()>0)//If there are points in the input
			srcptoff = mySource->pointOffset(mySourceNum);//get vertex information to generate point on the vertex
		mySourceNum++;//Move to the next point

	}
	GA_Offset ptoff = gdp->vertexPoint(vtxoff);//get birth particle information in the world
	myvariance.set(ptoff,UT_Vector3(myVarx,myVary,myVarz));//seting myVariance
	
	if (GAisValid(srcptoff))//if there is a valid vertex information from source geometry
	{
		if(mySource->getPointMap().indexSize()>0)//if there is valid vertex points from source geometry
			gdp->setPos3(ptoff, mySource->getPos3(srcptoff));//assign the pos of the vertex to the particle
		else
			gdp->setPos3(ptoff, UT_Vector3(ORIGINX(),ORIGINY(),ORIGINZ()));//else birth in origin

		if(mySourceVel.isValid())//if source geometry has valid velocity info for the vertex
			myVelocity.set(ptoff, mySourceVel.get(srcptoff)+(UT_Vector3(INITVELX(),INITVELY(),INITVELZ()) + UT_Vector3(myVarx,myVary,myVarz)));
		else//else set Manually of the vertex
			myVelocity.set(ptoff, UT_Vector3(INITVELX(),INITVELY(),INITVELZ()) + UT_Vector3(myVarx,myVary,myVarz));
	}
	else//if no source geometry provided
	{
		gdp->setPos3(ptoff, ORIGINX(),ORIGINY(),ORIGINZ());//generates the point in the origin point
		myVelocity.set(ptoff,(UT_Vector3(INITVELX(),INITVELY(),INITVELZ()) + UT_Vector3(myVarx,myVary,myVarz)));
	}

	//first component tells how long they have been alive. (Set to 0)
	my_global_id.set(ptoff,id);
	myLife.set(ptoff,0,0);
	fpreal lifemin = LIFEMIN();
	fpreal lifemax = LIFEMAX();
	double random = SYSdrand48();
	fpreal dif = lifemax - lifemin;
	ptclLifeSpan = (random * dif) + lifemin;
	//set when particles are going to die (in frames)
	myLife.set(ptoff,1,ptclLifeSpan);
}

//Euler integration
int 
SGParticleGen::Move(GA_Offset ptoff, const UT_Vector3 &force)
{
	float life = myLife.get(ptoff,0);
	float death = myLife.get(ptoff,1);
	life += 1;
	myLife.set(ptoff,0,life);
	myage.set(ptoff,life);
	
	if(life >= death)
		return 0;
	float h = 1./30;
	UT_Vector3 vel = myVelocity.get(ptoff);
	vel = vel + h*force;
	myVelocity.set(ptoff,vel);

	UT_Vector3 pos = gdp->getPos3(ptoff);
	pos  = pos + h*vel;
	gdp->setPos3(ptoff,pos);

	return 1;
} 

UT_Vector3 
SGParticleGen::steer(GA_Offset ptoff,UT_Vector3 force, fpreal now)
{
	fpreal rP = BSIZE();
	fpreal rS = COLRAD();
	fpreal R = rS + rP;
	UT_Vector3 colCen = UT_Vector3(COLPOSX(now),COLPOSY(now),COLPOSZ(now));
	UT_Vector3 vI  = myVelocity.get(ptoff);
	UT_Vector3 nVI = vI;
	nVI.normalize();
	UT_Vector3 xI  = gdp->getPos3(ptoff);
	UT_Vector3 xIS = colCen - xI ;

	fpreal sClose = dot(xIS,nVI);
	if (sClose < 0)
		return UT_Vector3(0,0,0);
	fpreal tC = TC()/30.0;
	fpreal dC = vI.length()*tC;

	if (sClose > dC)
		return  UT_Vector3(0,0,0);

	UT_Vector3 xClose = xI + sClose*nVI;
	fpreal d = (xClose-colCen).length();
	if (d > R)
		return  UT_Vector3(0,0,0);
	
	UT_Vector3 vC = xClose - colCen;
	UT_Vector3 nVC = vC;
	nVC.normalize();
	
	UT_Vector3 xT = colCen+ R*nVC;
	fpreal dT = (xT-xI).length();
	fpreal vT = dot(vI,(xT-xI))/(1.0*dT);
	fpreal tT = dT/vT;

	fpreal vS = (cross(nVI,(xT-xI))).length()/(1.0*tT);
	fpreal aS = 2*vS/(1.0*tT);
	fpreal e  = dot(nVC, force); 
	UT_Vector3 aRes = max((double)(aS-e),0.0)*nVC; 

	return aRes;
}


//Interparticle forces
UT_Vector3 
SGParticleGen::localForces(GA_Size i, UT_Vector3 force, fpreal now)
{
	UT_Vector3 f(0,0,0);
	UT_Vector3 goal = UT_Vector3(GOALX(),GOALY(),GOALZ());
	UT_Vector3 aA, aAI,nA,aV,aVI,nV,aC,aCI,nC , aG,aGI, nG, aCol,nCol;
	fpreal ka = KA();
	fpreal kv = KV();
	fpreal kc = KC();
	fpreal kG = KG();
	fpreal aR = MAXACC();
	fpreal fAngle = FANGLE();
	fpreal pAngle = PANGLE();
	aGI = UT_Vector3(0,0,0);
	aAI = UT_Vector3(0,0,0);
	aVI = UT_Vector3(0,0,0);
	aCI = UT_Vector3(0,0,0);
	UT_Vector3 aColI = UT_Vector3(0,0,0);
	UT_Vector3 result = UT_Vector3(0,0,0);
	GA_Size leader = 1;
	for ( GA_Size j = 0; j < my_gpp->getNumParticles(); j++)
	{
		if( i != j)
		{
			GA_Offset offset = my_gpp->vertexPoint(j);
			GA_Offset off0 = my_gpp->vertexPoint(i);
			GA_Offset offL = my_gpp->vertexPoint(leader);//leader particle

			UT_Vector3 pI = gdp->getPos3(off0);//position of point to be considered
			UT_Vector3 pJ = gdp->getPos3(offset);//position of point to be compared with		
			UT_Vector3 p1 = gdp->getPos3(offL);//leader particle


			UT_Vector3 vI = myVelocity.get(off0);//velocity of point to be considered
			UT_Vector3 vJ = myVelocity.get(offset);//velopcity of point to be compared with

			UT_Vector3 pIJ = pJ-pI;//vector between the points
			UT_Vector3 nIJ = pIJ;
			nIJ.normalize();//normalized vector PIJ
			fpreal d = pIJ.length();

			UT_Vector3 pIG = goal - pI;
			UT_Vector3 nIG = pIG;
			nIG.normalize();
			fpreal r = pIG.length();

			UT_Vector3 pI1 = p1 - pI;
			UT_Vector3 nI1 = pI1;
			nI1.normalize();
			fpreal leadDist = pI1.length();
			
			
			fpreal kD, kTheta;
			fpreal kL = LEADINF();
			//Influence calc based on Distance
			if ((d > (INFRAD()+BSIZE())) )
				kD = 0;
			else if ( d < (BSIZE()*2))
				kD = 1;
			else 
				kD = (INFRAD()-d)/(INFRAD()-BSIZE()); 
				
			// Influence calc based on FOV
			fpreal theta = dot((vI/vI.length()),nIJ);
			kTheta = (cos(pAngle)-theta)/(cos(pAngle)-cos(fAngle));
			if (kTheta > 1.0)
				kTheta = 1;
			if (kTheta < 0.0)
				kTheta = 0;

			
			aCol = steer(off0,force,now);
			aColI += kTheta*aCol;
			mySteer.set(off0,aColI);

			if (i==leader)
			{
				aG = (kG/(r-BSIZE()))*nIG;//Goal Force
			}
			else
			{
				aG = (1.0 - kL)*(kG/(r-BSIZE()))*nIG + (kL/(leadDist-BSIZE()))*nI1;
			}
			
			aGI += kTheta*aG;

			aA = -(ka/(d-BSIZE()))*nIJ;//avoidance force
			aAI += kTheta*kD*aA;

			aV = kv*(vJ-vI);//Velocity matching force
			aVI += kTheta*kD*aV;
			
			aC = kc*pIJ;//Centering force
			aCI += kTheta*kD*aC;
			
		}

	}

	aR = MAXACC();
	
	nCol = aColI;
	nCol.normalize();
	f = min((float)aR,(float)aColI.length())*nCol;
	aR = MAXACC() - f.length();

	nG = aGI;
	
	nG.normalize();
	f = f + min((float)aR,(float)aGI.length())*nG;
	aR = MAXACC() - f.length();

	nA = aAI;
	
	nA.normalize();
	f = f +  min((float)aR,(float)aAI.length())*nA;
	aR = MAXACC() - f.length();

	nV = aVI;
	
	nV.normalize();
	f = f + min((float)aR,(float)aVI.length())*nV;
	aR = MAXACC() - f.length();

	nC = aCI;
	
	nC.normalize();
	f = f + min((float)aR,(float)aCI.length())*nC;
	result = f;
	return result;
}

UT_Vector3 
SGParticleGen::calcForces(GA_Offset ptoff, GA_Size j, fpreal now)
{
	//Global Forces
	UT_Vector3 grav = UT_Vector3(GRAVX(),GRAVY(),GRAVZ());
	UT_Vector3 wind = UT_Vector3(WINDX(),WINDY(),WINDZ());
	UT_Vector3 pos = gdp->getPos3(ptoff);

	fpreal32 d = AIRRES();
	UT_Vector3 result;
	UT_Vector3 temp = myVelocity.get(ptoff);
	fpreal32 x = GRAVX() + d*(WINDX() - temp[0]);
	fpreal32 y = GRAVY() + d*(WINDY() - temp[1]);
	fpreal32 z = GRAVZ() + d*(WINDZ() - temp[2]);
	UT_Vector3 f = UT_Vector3(x,y,z);
	UT_Vector3 localF = localForces(j,f,now);
	
	result = UT_Vector3(x,y,z) + localF;
	return result;
}


void
SGParticleGen::timeStep(fpreal now)
{
	int nbirth = BIRTH(now);//BirthRate
	GA_Size maxPart = MAXNUM();//Max Particles
	if(error() >= UT_ERROR_ABORT)//if user Cancels the cooking process
	return;
	GA_ElementGroup* pointgroup = gdp->createInternalElementGroup(GA_ATTRIB_POINT);
	//Birthing Particles and assigning initial state
	for(int i = 0; i < nbirth; ++i)
	{
		if(my_gpp->getNumParticles() <= maxPart)
			Emit(i);
	}

	//Updating each particle
	for(GA_Size i=0; i< my_gpp->getNumParticles();i++)
	{
		UT_Vector3 force = calcForces(my_gpp->vertexPoint(i), i, now );
		
		if(!Move(my_gpp->vertexPoint(i),force))
		{
			my_gpp->deadParticle(i);//mark as dead
		}
	}
	my_gpp->deleteDead();//clear deadParticles
	gdp->destroyUnusedPoints();//remove from Viewport
	
}

void 
SGParticleGen::initSystem()
{
	if(!gdp) gdp = new GU_Detail;

	if(gdp->getPointMap().indexSize() > 0 || myVelocity.isInvalid())
	{
		mySourceNum = 0;
		gdp->clearAndDestroy();
		my_gpp = (GEO_PrimParticle *)gdp->appendPrimitive(GEO_PRIMPART);
		my_gpp->clearAndDestroy();
		//generating an array of attributes for the particle system.
		myVelocity = GA_RWHandleV3(gdp->addFloatTuple(GA_ATTRIB_POINT,"v",3));
		if(myVelocity.isValid())
			myVelocity.getAttribute()->setTypeInfo(GA_TYPE_VECTOR);
		myLife = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT,"life",2));
		myage = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT,"age",1));
		my_global_id = GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
		myvariance = GA_RWHandleV3(gdp->addFloatTuple(GA_ATTRIB_POINT,"variance",3));
		if(myvariance.isValid())
			myvariance.getAttribute()->setTypeInfo(GA_TYPE_VECTOR);
		mySteer = GA_RWHandleV3(gdp->addFloatTuple(GA_ATTRIB_POINT,"steer",3));
		if(mySteer.isValid())
			mySteer.getAttribute()->setTypeInfo(GA_TYPE_VECTOR);
		mylforce = GA_RWHandleV3(gdp->addFloatTuple(GA_ATTRIB_POINT,"local Accel",3));
		if(mylforce.isValid())
			mylforce.getAttribute()->setTypeInfo(GA_TYPE_VECTOR);
	}
}

OP_ERROR 
SGParticleGen::cookMySop(OP_Context &context)
{
	OP_AutoLockInputs inputs(this);
	if(inputs.lock(context) >= UT_ERROR_ABORT)
		return error();
	
	OP_Node::flags().timeDep = 1;

	CH_Manager *chman = OPgetDirector()->getChannelManager();

	fpreal currframe = chman->getSample(context.getTime());
	fpreal reset = STARTFRAME();

	if (currframe <= reset || !my_gpp)
	{
		my_lastCookTime = reset;
		initSystem();

	}
	else
	{
		mySource = inputGeo(0,context); // try mySource = gdp
		if(mySource)
		{
			mySourceVel = GA_ROHandleV3(mySource->findFloatTuple(GA_ATTRIB_POINT, "v", 3));

			if(mySourceVel.isInvalid())
				mySourceVel = GA_ROHandleV3(mySource->findFloatTuple(GA_ATTRIB_POINT,"N",3));
		}
		
		notifyGroupParmListeners(0,-1,mySource,NULL);

		currframe += 0.05;//tolerance
		while(my_lastCookTime < currframe)
		{
			timeStep(chman->getTime(my_lastCookTime));
			my_lastCookTime += 1;
		}

		select(GU_SPoint);
	}

	return error();
}

const char *
SGParticleGen::inputLabel(unsigned inum) const
{
    switch (inum)
    {
	case 0: return "Particle Source Geometry";
	case 1:	return "Collision Object";
    }
    return "Unknown source";
}