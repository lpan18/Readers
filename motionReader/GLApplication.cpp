#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <float.h>

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#include "AdjustableCamera.h"
#include "BoneNode.h"

using namespace std;

//Global variables:
//ControllableCamera camera;//Our camera in the virtual world
AdjustableCamera camera;
bool bShowFramerate = false; // by default, we want the current fps displayed
BoneNode* rootBone = NULL;
int total_animation_frames = 0;
float currentFrame = 0;
int currentlySetFramerate = 120; //i.e. how many animation frames we should display per second
bool bInterpolateAnimations = true;
bool bCurrentlyPlayingAnimation = false;

float NEAR_CLIPPING_PLANE = 0.1f;
float FAR_CLIPPING_PLANE  = 100000.0f;
int DEFAULT_LINE_WIDTH  = 3;
const float DEGREES_PER_RADIAN = (180.0f/3.1415926f);
const float RADIANS_PER_DEGREE = (3.1415926f/180.0f);

//Function prototypes
void init();
void display();
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);
void specialKeyboard(int key, int x, int y);
void stopAndResetAnimation();
void pauseAnimation();
void resumeAnimation();
void recursivelyResetAllNodes(BoneNode* node);
long getTime(){return clock();}
void calculateAndShowFPS();
void drawLine( float ax, float ay, float az, float bx, float by, float bz, int r = 0, int g = 0, int b = 0, int a = 255, int width = DEFAULT_LINE_WIDTH);
void getNodeMaxMinOffsets(BoneNode* node, float& xmax, float& ymax, float& zmax, float& xmin, float& ymin, float& zmin, float xoff, float yoff, float zoff);
float distanceBetweenPoints(float p1x, float p1y, float p1z, float p2x, float p2y, float p2z);
float maxFloat(vector<float> values);
float minFloat(vector<float> values);
void placeCameraAtGoodViewDistance(BoneNode* rootBone);
BoneNode* buildBoneTreeFromBVHFile(std::string filename);
BoneNode* parseBoneNode(ifstream& fin, string node_name);
int parseMotionData(ifstream& fin, BoneNode* rootBone);
void addAllNodesToList(BoneNode* node, std::vector<BoneNode*>& bonelist);
void parseMotionDataForIndividualNode(BoneNode* node, std::istringstream& sin);
void writeOutAnimatedSkeletonToFile(string outputFilename, BoneNode* rootNode, unsigned int frame_count);
void recursivelyWriteOutNode(ofstream& fout, BoneNode* node, bool isRootNode, unsigned int tab_level);
void recursivelyWriteOutNodeChannelData(ofstream& fout, BoneNode* node, unsigned int frame);
void updateNodeParametersWithoutInterpolation(BoneNode* node, float interpolatedFrame);
void updateNodeParameters(BoneNode* node, float interpolatedFrame);
void drawHierarchy(BoneNode* node);

void idle();

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB| GLUT_DEPTH);
	glutInitWindowSize (800, 600);
	glutInitWindowPosition (100, 50);
	glutCreateWindow("CMPUT411 Assn2: Animation");

	init ();
   
	//Where the bone-loading code comes in
	string fileToBeLoaded = "test.bvh";
	if(argc > 1){
		fileToBeLoaded = argv[1];
	}
	rootBone = buildBoneTreeFromBVHFile(fileToBeLoaded);
	if(rootBone == NULL){
		std::cout<<"Was not able to load bone model from file \""<<fileToBeLoaded<<"\"!\n"<<std::flush;
	}

	placeCameraAtGoodViewDistance(rootBone);

	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialKeyboard);
	glutMainLoop();
	return 0;
}

void init(void)//Initialzation stuff
{
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
}

void idle()
{
	//We now want to compute how much time has elapsed since the last time this function was called
	static long timeOfPreviousCall = glutGet(GLUT_ELAPSED_TIME);//Note: because this variable is static, it will only be set the first time this function runs
	long currentTime = glutGet(GLUT_ELAPSED_TIME);
	if((currentTime - timeOfPreviousCall) == 0){
		return;
	}

	double elapsed_time_in_seconds = ((double)(currentTime - timeOfPreviousCall))/((double)1000.0);
	timeOfPreviousCall = currentTime;


	//Use the amount of elapsed time to determine what frame we should be on (if the animation is playing)
	if(bCurrentlyPlayingAnimation){
		currentFrame += (elapsed_time_in_seconds)*((double)currentlySetFramerate);
	}
	if(currentFrame >= total_animation_frames -1){
		currentFrame = 0;
	} else if( currentFrame < 0){
		if(currentlySetFramerate < 0){//i.e. We're going backwards
			currentFrame = total_animation_frames-1.01f;
		}
		else {
			currentFrame = 0;
		}
	}
	glutPostRedisplay();
}

void display()
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glColor3f (1.0, 1.0, 1.0);
	glLoadIdentity ();
	//calculateAndShowFPS();
	
	camera.updateCamera();//Handles all the camera-related projection set-up stuff for this render

	drawLine(0,0,0, 10,0,0, 255,0,0,127, 1);//BONUS feature: renders a little XYZ axis at the origin. Helpful orientation purposes.
	drawLine(0,0,0, 0,10,0, 0,255,0,127, 1);
	drawLine(0,0,0, 0,0,10, 0,0,255,127, 1);

	//Update the skeleton pose if the animation is playing (i.e. update the translation and rotation of every joint)
	if(bCurrentlyPlayingAnimation){
		if(bInterpolateAnimations){
			updateNodeParameters(rootBone, currentFrame);
		}else{
			updateNodeParametersWithoutInterpolation(rootBone,currentFrame);//Little extra debugging feature
		}
	}

	drawHierarchy(rootBone);//Draw the Skeleton in whatever pose it has been set in
	
	glutSwapBuffers();// Enables Double buffering
}

void reshape (int w, int h)
{
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90,((float)(w)/(float)(h)),NEAR_CLIPPING_PLANE,FAR_CLIPPING_PLANE);//i.e. use an FOV of 90 degrees for this assignment
    glMatrixMode (GL_MODELVIEW);
	return;
}

void keyboard(unsigned char key, int x, int y)
{
	camera.keyInput(key);//The camera also has its own keyboard controls that it listens for
    switch (key)
    {
	    case 'q': exit(0); break;
		case 27: exit(0); break;//(i.e. Esc key)
		case 'w': writeOutAnimatedSkeletonToFile("output.bvh", rootBone, total_animation_frames); break;
		case '-': currentlySetFramerate -=10; break;
		case '+': currentlySetFramerate +=10; break;
		case ' ': bInterpolateAnimations = !bInterpolateAnimations; break;// Cool little extra feature
		case 's': stopAndResetAnimation(); break;
		case 'p': resumeAnimation(); break;
		case 'P': pauseAnimation(); break;
	 }
}

void specialKeyboard(int key, int x, int y)//This handles the special keys like the arrow keys
{	
	camera.specialKeys(key);//The camera is interested in the arrow keys for camera movement
}

void stopAndResetAnimation(){
	bCurrentlyPlayingAnimation = false;
	recursivelyResetAllNodes(rootBone);
	currentFrame = 0;
	currentlySetFramerate = 120;
}

//Sets this rotations and translation of this joint and all its children back to zero
void recursivelyResetAllNodes(BoneNode* node){
	node->xTranslation = node->yTranslation = node->zTranslation = 0;
	node->xRotation = node->yRotation = node->zRotation = 0;
	node->refreshQuaternionRepresentation();
	for(unsigned int i = 0; i<node->childrenNodes.size(); i++){
		recursivelyResetAllNodes(node->childrenNodes[i]);
	}
}

void pauseAnimation(){
	bCurrentlyPlayingAnimation = false;
}

void resumeAnimation(){
	bCurrentlyPlayingAnimation = true;
}

void calculateAndShowFPS()//BONUS feature: helps to know what your display framerate is on your current 
{
	static int renders = 0;//Helps count fps
	static long lastCheckin = 0;
	renders++;

	if(getTime()> lastCheckin + CLOCKS_PER_SEC && bShowFramerate)//displays current framerate
	{
		lastCheckin = getTime();
		cout<<renders<<endl;
		renders=0;
	}
}

//Useful utility function for drawing lines in 3D space
void drawLine( float ax, float ay, float az, float bx, float by, float bz, int r, int g, int b, int a, int width)
{
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4ub( r, g, b, a);

    glLineWidth(width);
    glBegin(GL_LINES);
    glVertex3f( ax, ay, az);
    glVertex3f( bx, by, bz);
    glEnd();

    glDisable(GL_BLEND);
    glEnable(GL_TEXTURE_2D);
}

//A rough technique for placing the camera at a good viewing distance from the animation
void placeCameraAtGoodViewDistance(BoneNode* rootBone){

	//First, determine the radius of a boudning sphere around the skeleton centered at the root node
	float maxTotalOffsetX = 0, maxTotalOffsetY = 0, maxTotalOffsetZ = 0;
	float minTotalOffsetX = 0, minTotalOffsetY = 0, minTotalOffsetZ = 0;
	getNodeMaxMinOffsets(rootBone,maxTotalOffsetX,maxTotalOffsetY,maxTotalOffsetZ,minTotalOffsetX,minTotalOffsetY,minTotalOffsetZ,0,0,0);
	float distance1 = distanceBetweenPoints(0,0,0,maxTotalOffsetX,maxTotalOffsetY,maxTotalOffsetZ);
	float distance2 = distanceBetweenPoints(0,0,0,minTotalOffsetX,minTotalOffsetY,minTotalOffsetZ);
	float radiusOfModelBoundingSphere = distance1;
	if(distance2 > radiusOfModelBoundingSphere){
		radiusOfModelBoundingSphere = distance2;
	}

	//Next, determine the minimum Axis-Aligned Bounding Box that encompasses all the translations that the root node makes
	float midpointX = 0, midpointY = 0, midpointZ = 0;
	float radiusOfTotalSceneBoundingSphere = radiusOfModelBoundingSphere;
	
	if(rootBone->xTranslations.size() > 0){

		float maxTranslationX = maxFloat(rootBone->xTranslations);
		float maxTranslationY = maxFloat(rootBone->yTranslations);
		float maxTranslationZ = maxFloat(rootBone->zTranslations);

		float minTranslationX = minFloat(rootBone->xTranslations);
		float minTranslationY = minFloat(rootBone->yTranslations);
		float minTranslationZ = minFloat(rootBone->zTranslations);

		float radiusOfAnimationBoundSphere = distanceBetweenPoints(maxTranslationX,maxTranslationY,maxTranslationZ,
			minTranslationX,minTranslationY,minTranslationZ) / 2.0f;

		midpointX = (maxTranslationX + minTranslationX) / 2.0f;
		midpointY = (maxTranslationY + minTranslationY) / 2.0f;
		midpointZ = (maxTranslationZ + minTranslationZ) / 2.0f;

		radiusOfTotalSceneBoundingSphere = radiusOfAnimationBoundSphere + radiusOfModelBoundingSphere;
	}

	//Assuming a FOV of 90, and given that tan(45 degrees) = 1, then viewDistance = radiusOfTotalSceneBoundingSphere / tan(45), so
	float cameraDistanceFromMidpoint = radiusOfTotalSceneBoundingSphere;

	camera.x = midpointX;
	camera.y = midpointY;
	camera.z = midpointZ + cameraDistanceFromMidpoint;

	camera.yaw = 0;//(i.e. set these to make camera look in it's default -Z direction)
	camera.pitch = 0;
	camera.roll = 0;
}

//Gets the max/min offsets from a joint: useful for find the outbounds of the starting skeleton
void getNodeMaxMinOffsets(BoneNode* node, float& xmax, float& ymax, float& zmax, float& xmin, float& ymin, float& zmin, float xoff, float yoff, float zoff)
{
	xoff += node->xOffset;
	yoff += node->yOffset;
	zoff += node->zOffset;

	if(xoff > xmax){
		xmax = xoff;
	}
	if(yoff > ymax){
		ymax = yoff;
	}
	if(zoff > zmax){
		zmax = zoff;
	}
	if(xoff < xmin){
		xmin = xoff;
	}
	if(yoff < ymin){
		ymin = yoff;
	}
	if(zoff < zmin){
		zmin = zoff;
	}

	for(unsigned int i = 0; i < node->childrenNodes.size(); i++){
		getNodeMaxMinOffsets(node->childrenNodes[i],xmax,ymax,zmax,xmin,ymin,zmin,xoff,yoff,zoff);
	}
}

float distanceBetweenPoints(float p1x, float p1y, float p1z, float p2x, float p2y, float p2z){
	return sqrtf((p2x-p1x)*(p2x-p1x) + (p2y-p1y)*(p2y-p1y) + (p2z-p1z)*(p2z-p1z));
}

float maxFloat(vector<float> values){
	float max = FLT_MIN;
	for(unsigned int i = 0; i<values.size(); i++){
		if(values[i]>max)
			max = values[i];
	}
	return max;
}

float minFloat(vector<float> values){
	float min = FLT_MAX;
	for(unsigned int i = 0; i<values.size(); i++){
		if(values[i]<min)
			min = values[i];
	}
	return min;
}

//Parses and BVH file to construct a skeleton heirarchy
BoneNode* buildBoneTreeFromBVHFile(std::string filename)
{
	std::ifstream fin;
	fin.open(filename.c_str(),std::ios::in);
	BoneNode* rootBoneNode = NULL;

	if(!fin){
		std::cout<<"WARNING! Mesh file \""<<filename<<"\" could not be found!\n";
		fin.close();
		return NULL;
	}

	string line;
	while(!fin.eof()){

		getline(fin, line);

		if(line == "" || line == "\n"){//i.e. this is a blank line
			continue;
		}

		std::istringstream sin(line);
		std::string firstToken;
		sin>>firstToken;

		if(firstToken == "HIERARCHY"){//i.e. check for a "comment" line
			//std::cout<<"Heirarchy Definition Line\n";
		}else if(firstToken == "ROOT" || firstToken == "JOINT"){
			std::string nodeName = "";
			sin>>nodeName;
			rootBoneNode = parseBoneNode(fin,nodeName);
		}else if(firstToken == "MOTION"){
			total_animation_frames = parseMotionData(fin,rootBoneNode);
			break;
		}else{
			std::cout<<"WARNING UNKNOWN LINE\n";
		}
	}
	
	fin.close();
	return rootBoneNode;
}

//Parses a joint from the current position in the file stream (acts recursively if this joint contains another joint)
BoneNode* parseBoneNode(ifstream& fin, string node_name){
	BoneNode* newBone = new BoneNode;
	newBone->name = node_name;

	string line;
	int channel_count = 0;

	while(!fin.eof()){
		getline(fin, line);

		if(line == "" || line == "\n"){//i.e. this is a blank line
			continue;
		}

		std::istringstream sin(line);
		std::string firstToken;
		sin>>firstToken;

		if(firstToken == "{"){
			//std::cout<<"Start of node Indicator\n";

		}else if(firstToken == "OFFSET"){
			sin>>newBone->xOffset;
			sin>>newBone->yOffset;
			sin>>newBone->zOffset;

		}else if(firstToken == "CHANNELS"){
			sin>>channel_count;
			newBone->channels.clear();
			for(int i = 0; i<channel_count; i++){
				newBone->channels.push_back("");
				sin>>(newBone->channels[i]);
			}

		}else if(firstToken == "JOINT"){
			string childName = "";
			sin>>childName;
			newBone->childrenNodes.push_back(parseBoneNode(fin,childName));

		}else if(firstToken == "End" || firstToken == "end" || firstToken == "END" ){
			string childName = "End Site";
			newBone->childrenNodes.push_back(parseBoneNode(fin,childName));

		}else if(firstToken == "}"){
			break;//i.e. We're done loading this node

		}else{
			std::cout<<"WARNING UNKNOWN LINE\n";
		}
	}
	return newBone;
}

//Parses the motion data section of a BVH file, and fills out a joint tree
int parseMotionData(ifstream& fin, BoneNode* rootBone){
	
	if(rootBone == NULL){
		std::cout<<"ERROR! No bone hierarchy is present!";
	}

	std::vector<BoneNode*> nodeList;
	addAllNodesToList(rootBone, nodeList);//Creates an ordered list of all the bone nodes

	unsigned int total_frames_expected = 0;
	unsigned int total_frames_recorded = 0;
	double proposedFrameTime = 0;
	string line;

	while(!fin.eof()){
		getline(fin, line);

		if(line == "" || line == "\n"){//i.e. this is a blank line
			continue;
		}

		std::istringstream sin(line);
		std::string firstToken;
		sin>>firstToken;

		if(firstToken == "Frames:"){
			sin>> total_frames_expected;

		}else if(firstToken == "Frame"){
			std::string secondToken;
			sin>>secondToken;
			if(secondToken == "Time:"){
				sin>>proposedFrameTime;
			}
		}else{//Otherwise, assume that motion data is on this line
			std::istringstream lineStream(line);
			for(unsigned int i = 0; i < nodeList.size(); i++){
				parseMotionDataForIndividualNode(nodeList[i], lineStream);
			}
			total_frames_recorded++;
		}
	}

	if(total_frames_recorded != total_frames_expected){
		std::cout<<"WARNING! Expected "<<total_frames_expected<<" frames, but actually found "<<total_frames_recorded<<" frames instead!"<<std::endl;
	}

	return total_frames_recorded;
}

//Fills a single joint with the motion data from the current location in the filestream
void parseMotionDataForIndividualNode(BoneNode* node, std::istringstream& sin){
	string currentChannel;

	float value = 0;
	for(unsigned int i = 0; i<node->channels.size(); i++){
		currentChannel = node->channels[i];

		if(currentChannel == "Xposition"){
			sin>>value;
			node->xTranslations.push_back(value);
		}else if(currentChannel == "Yposition"){
			sin>>value;
			node->yTranslations.push_back(value);
		}else if(currentChannel == "Zposition"){
			sin>>value;
			node->zTranslations.push_back(value);
		}else if(currentChannel == "Zrotation"){
			sin>>value;
			node->zRotations.push_back(value);
		}else if(currentChannel == "Yrotation"){
			sin>>value;
			node->yRotations.push_back(value);
		}else if(currentChannel == "Xrotation"){
			sin>>value;
			node->xRotations.push_back(value);
		}
	}
}

//Builds a list of all joints in a tree structure in order of a depth-first search
void addAllNodesToList(BoneNode* node, std::vector<BoneNode*>& bonelist){
	bonelist.push_back(node);
	for(unsigned int i = 0; i<node->childrenNodes.size(); i++){
		addAllNodesToList(node->childrenNodes[i],bonelist);
	}
}

//Writes out the HEIRARCHY and MOTION sections of a bvh
void writeOutAnimatedSkeletonToFile(string outputFilename, BoneNode* rootNode, unsigned int frame_count){

	ofstream fout(outputFilename.c_str());
	if(!fout || !fout.good()){
		cout<<"ERROR! Unable to open \""<<outputFilename<<"\" for writing!\n";
		return;
	}

	//First output the HIERARCHY
	fout<<"HIERARCHY"<<endl;
	recursivelyWriteOutNode(fout,rootNode,true,0);

	//Next, the motion Section:
	fout<<"MOTION"<<endl;
	fout<<"Frames: "<<frame_count<<endl;
	fout<<"Frame Time: .0083333"<<endl;//Hackish, just for this assignment

	for(unsigned int i = 0; i< frame_count; i++){
		recursivelyWriteOutNodeChannelData(fout, rootNode, i);
		fout<<endl;
	}

	//Finally, close the file
	fout.close();
}

//Writes out a joint node to a file and recursively calls its children as well
void recursivelyWriteOutNode(ofstream& fout, BoneNode* node, bool isRootNode, unsigned int tab_level){

	string whiteSpace = "";//This ensures a readable indentation for the file
	for(unsigned int i = 0; i<tab_level; i++){
		whiteSpace += "\t";
	}
	string extraWhiteSpace = whiteSpace + "\t";

	//First output the node's name
	fout<<whiteSpace;

	if(node->name == "End Site"){
		fout << node->name;
	}else{
		if(isRootNode){
			fout<<"ROOT";
		}else{
			fout<<"JOINT";
		}
		fout<<" "<<node->name;
	}
	fout<<endl;

	//Next, output the open bracket for this node
	fout<<whiteSpace<<"{"<<endl;
	
	//Then, the offsets
	fout<<extraWhiteSpace<<"OFFSET"<<" "<<node->xOffset<<" "<<node->yOffset<<" "<<node->zOffset<<endl;

	//Then, the Channels (but only if there are any)
	if(node->channels.size() > 0){
		fout<<extraWhiteSpace<<"CHANNELS"<<" "<<node->channels.size();
		for(unsigned int i = 0; i<node->channels.size(); i++){
			fout<<" "<<node->channels[i];
		}
		fout<<endl;
	}
	
	//Then, all the children nodes
	for(unsigned int i = 0; i< node->childrenNodes.size(); i++){
		recursivelyWriteOutNode(fout,node->childrenNodes[i],false,tab_level +1);
	}

	//Finally, output the close-bracket
	fout<<whiteSpace<<"}"<<endl;
}

//For a single frame, writes out the motion data for a whole node
void recursivelyWriteOutNodeChannelData(ofstream& fout, BoneNode* node, unsigned int frame){
	
	if(frame < node->xTranslations.size()){//i.e. this node has translation channels
		fout<<node->xTranslations[frame]<<" "<<node->yTranslations[frame]<<" "<<node->zTranslations[frame]<<" ";
	}
	if(frame < node->xRotations.size()){//i.e. this node has rotation channels
		fout<<node->zRotations[frame]<<" "<<node->yRotations[frame]<<" "<<node->xRotations[frame]<<" ";
	}

	//Then, do this for all the children nodes
	for(unsigned int i = 0; i< node->childrenNodes.size(); i++){
		recursivelyWriteOutNodeChannelData(fout,node->childrenNodes[i],frame);
	}
}

//Debugging function: update the skeleton pose using no interpolation
void updateNodeParametersWithoutInterpolation(BoneNode* node, float interpolatedFrame)
{
	unsigned int currentFrame = ((unsigned int)interpolatedFrame);

	if(((unsigned int)currentFrame) < node->xTranslations.size()){
		node->xTranslation = node->xTranslations[currentFrame];
		node->yTranslation = node->yTranslations[currentFrame];
		node->zTranslation = node->zTranslations[currentFrame];
	}

	if(((unsigned int)currentFrame) < node->xRotations.size()){
		node->xRotation = node->xRotations[currentFrame];
		node->yRotation = node->yRotations[currentFrame];
		node->zRotation = node->zRotations[currentFrame];
		node->refreshQuaternionRepresentation();
	}

	for(unsigned int i = 0; i < node->childrenNodes.size(); i++){
		updateNodeParametersWithoutInterpolation(node->childrenNodes[i], interpolatedFrame);
	}
}

//Update the pose of the skeleton (rotation and translation of all joints), interpolating between frames if necessary
void updateNodeParameters(BoneNode* node, float interpolatedFrame)
{
	unsigned int startFrame = ((unsigned int)interpolatedFrame);
	unsigned int endFrame = startFrame +1;
	float t = interpolatedFrame - ((float)startFrame);

	if( endFrame < node->xTranslations.size()){//i.e. this Node has translation channels
		float from_x = node->xTranslations[startFrame];
		float from_y = node->yTranslations[startFrame];
		float from_z = node->zTranslations[startFrame];

		float to_x = node->xTranslations[endFrame];
		float to_y = node->yTranslations[endFrame];
		float to_z = node->zTranslations[endFrame];

		//Lerp the translations
		float oneMinust = (1.0f - t);
		node->xTranslation = oneMinust*from_x + t*to_x;
		node->yTranslation = oneMinust*from_y + t*to_y;
		node->zTranslation = oneMinust*from_z + t*to_z;
	}

	if(((unsigned int)currentFrame) < node->xRotations.size()){//i.e. this node has rotation channels
		
		//To interpolate rotations correctly, we need to compose both the start and destination rotations into seperate quaterions first
		float from_x_rot = node->xRotations[startFrame];
		float from_y_rot = node->yRotations[startFrame];
		float from_z_rot = node->zRotations[startFrame];

		float from_qw = 0, from_qx = 0, from_qy = 0, from_qz = 0;
		BoneNode::convertEulerAnglesToQuaternion(from_x_rot*RADIANS_PER_DEGREE,from_y_rot*RADIANS_PER_DEGREE,from_z_rot*RADIANS_PER_DEGREE,
			from_qw,from_qx,from_qy,from_qz);

		float to_x_rot = node->xRotations[endFrame];
		float to_y_rot = node->yRotations[endFrame];
		float to_z_rot = node->zRotations[endFrame];

		float to_qw = 0, to_qx = 0, to_qy = 0, to_qz = 0;
		BoneNode::convertEulerAnglesToQuaternion(to_x_rot*RADIANS_PER_DEGREE,to_y_rot*RADIANS_PER_DEGREE,to_z_rot*RADIANS_PER_DEGREE,
			to_qw,to_qx,to_qy,to_qz);
		
		//Finally, we then SLERP the start and destination quaternions together using the value 't'
		BoneNode::quaternionSlerp(from_qw,from_qx,from_qy,from_qz,to_qw,to_qx,to_qy,to_qz,t,node->qw,node->qx,node->qy,node->qz);
	}

	for(unsigned int i = 0; i < node->childrenNodes.size(); i++){
		updateNodeParameters(node->childrenNodes[i], interpolatedFrame);
	}
}

//Draws out the hierarchy of the skeleton recursivly utilizing the OpenGL matrix stack to help
void drawHierarchy(BoneNode* node)
{
	if(node == NULL){return;}//Sanity check

	glPushMatrix();{//This will ensure that if a joint has multiple children we will pop back to the correct matrix when it comes time to render that child

		glTranslatef(node->xOffset, node->yOffset, node->zOffset);//Third, do the base offset translation (remember: Matrix actions are in reverse order!)		
		glTranslatef(node->xTranslation, node->yTranslation, node->zTranslation);//Second, do any channel-related translations

		float angle_radians=0, axis_x=0, axis_y=0, axis_z=0;
		node->convertQuaternionToAngleAndAxis(node->qw,node->qx,node->qy,node->qz,angle_radians,axis_x,axis_y,axis_z);
		glRotatef((angle_radians*(180.0f/3.14159f)), axis_x, axis_y, axis_z);//First, do any channel-related rotations


		for(unsigned int i = 0; i<node->childrenNodes.size(); i++){//Recursively draw all child bones
			BoneNode* child = node->childrenNodes[i];
			if(child != NULL){
				drawLine(0,0,0,child->xOffset,child->yOffset,child->zOffset);
				drawHierarchy(child);
			}
		}
		glPopMatrix();//Pop the current matrix so that as the recursion unwinds, each level has the appropriate matrix on the stack
	}
}
