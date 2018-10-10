#include <iostream>
#include <vector>
#include <fstream> 
#include <sstream> 
#include <string>
#include <math.h>

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

using namespace std;

// Begin globals.
static int isWire = 1; // Is wireframe?
static unsigned int aModelList; // List index.
vector<unsigned int> numVerticesPerFace;
float *vertexArray, *movedVertexArray;
unsigned int *displayList;
int numVertices = 0;
float epsilon = 0.000001;
float centerX, centerY, centerZ;
bool orthoOrPers = true;
float angleX = 0, angleY = 0, angleZ = 0;
float unitX = 0, unitY = 0, unitZ = 0;

double eyeXO = 0, eyeYO = 0, eyeZO = 0;
double centerXO = 0, centerYO = 0, centerZO = -2.0;
double upXO = 0, upYO = 1, upZO = 0;
double eyeXP = 0, eyeYP = 0, eyeZP = 0.0;
double centerXP = 0, centerYP = 0, centerZP = -2.0;
double upXP = 0, upYP = 1, upZP = 0;
double angleLX = 0, angleLY = 0, angleLZ = 0;	//10*3.14159265/180 = 0.174532925

static bool isFog = true; // Is fog on?
static int fogMode = GL_LINEAR; // Fog mode.
static float fogStart = 1.0; // Fog start z value.
static float fogEnd = 5.0; // Fog end z value.

//for the first question, read in the .obj file
void Question1(char *fileName)
{
	vector<unsigned int> vertexIndices;
	vector<float> vertices;
	ifstream infile(fileName);
	string line;
	//only handle two cases
	while (getline(infile, line)) {
		//a line begins with v+space
		if(line[0] == 'v' && line[1] == ' ') {
			numVertices++;
			string str = line.substr(2, line.size()-1);
			istringstream iss(str); 
			float n;
			while (iss >> n) 
				vertices.push_back(n); 
		}
		//or begin with f+space
		else if(line[0] == 'f' && line[1] == ' ') {
			string str = line.substr(2, line.size()-1);
			istringstream iss(str);
			unsigned int n;
			int num = 0;
			while (iss >> n) {
				vertexIndices.push_back(n-1);
				num++;
			}
			numVerticesPerFace.push_back(num);
		}
	}
	infile.close();

	//dynamically allocate space for the arrays
	vertexArray = (float*)malloc(vertices.size()*sizeof(float));
	movedVertexArray = (float*)malloc(vertices.size()*sizeof(float));
	displayList = (unsigned int*)malloc(vertexIndices.size()*sizeof(unsigned int));
	for(unsigned int i = 0; i < vertices.size(); i++)
		vertexArray[i] = vertices[i];
	for(unsigned int i = 0; i < vertexIndices.size(); i++)
		displayList[i] = vertexIndices[i];
}

//write the model out to an .obj file, store the very original configuration
void Question2()
{
	FILE *fp = fopen("output.obj", "w");
	if(fp == NULL) {
		cout << "cannot write to .obj file" << endl;
		return ;
	}
	for(int i = 0; i < numVertices; i++)
		fprintf(fp, "v %f %f %f\n", vertexArray[i*3], vertexArray[i*3+1],
			vertexArray[i*3+2]);
	int faceBegIdx = 0;
	for(unsigned int i = 0; i < numVerticesPerFace.size(); i++) {
		if(numVerticesPerFace[i] == 3) {
			fprintf(fp, "f %d %d %d\n", displayList[faceBegIdx], 
				displayList[faceBegIdx+1], displayList[faceBegIdx+2]);
			faceBegIdx += 3;
		}
		else {
			fprintf(fp, "f %d %d %d %d\n", displayList[faceBegIdx], 
				displayList[faceBegIdx+1], displayList[faceBegIdx+2],
				displayList[faceBegIdx+3]);
			faceBegIdx += 4;
		}
	}
	fclose(fp);
}

void Question3()
{
	float totalX = 0, totalY = 0, totalZ = 0;
	float maxX = -999999, maxY = -999999, maxZ = -999999;
	float minX = 999999, minY = 999999, minZ = 999999;
	//compute the center as well as the scale
	for(int i = 0; i < numVertices*3; i++) {
		if(i%3 == 0) {
			totalX += vertexArray[i];
			if(vertexArray[i] > maxX)	maxX = vertexArray[i];
			if(vertexArray[i] < minX)	minX = vertexArray[i];
		}
		else if(i%3 == 1) {
			totalY += vertexArray[i];
			if(vertexArray[i] > maxY)	maxY = vertexArray[i];
			if(vertexArray[i] < minY)	minY = vertexArray[i];
		}
		else {
			totalZ += vertexArray[i];
			if(vertexArray[i] > maxZ)	maxZ = vertexArray[i];
			if(vertexArray[i] < minZ)	minZ = vertexArray[i];
		}
	}
	
	centerX = totalX/(float)numVertices;
	centerY = totalY/(float)numVertices;
	centerZ = totalZ/(float)numVertices;
	float scaleX = maxX-minX;
	float scaleY = maxY-minY;
	float scaleZ = maxZ-minZ;
	float scale = max(scaleX, scaleY);
	scale = max(scale, scaleZ);
	if(scale < epsilon)	
		scale = epsilon;
	centerX /= scale; centerY /= scale; centerZ /= scale;
	float offsetX = -centerX;
	float offsetY = -centerY;
	float offsetZ = -2-centerZ;
	for(int i = 0; i < numVertices*3; i++) {
		movedVertexArray[i] = vertexArray[i]/scale;
		if(i%3 == 0)
			movedVertexArray[i] += offsetX;
		else if(i%3 == 1)
			movedVertexArray[i] += offsetY;
		else
			movedVertexArray[i] += offsetZ;
	}
}

//switch between the pespective and orthographic projection
void Question5()
{
	if(orthoOrPers == true) {
		glLoadIdentity();
		//set the default camera: place at origin, looking at -z
		gluLookAt(eyeXO, eyeYO, eyeZO, centerXO, centerYO, 
			centerZO, upXO, upYO, upZO); 
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-1.0, 1.0, -1.0, 1.0, 1.0, 100.0);
		glMatrixMode(GL_MODELVIEW);
	}
	else {
		glLoadIdentity();
		//set the default camera: place at origin, looking at -z
		gluLookAt(eyeXP, eyeYP, eyeZP, centerXP, centerYP, 
			centerZP, upXP, upYP, upZP); 
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(90, 1, 1.0, 100);
		glMatrixMode(GL_MODELVIEW);
	}
}

// Drawing (display) routine.
void drawScene(void)
{
	float fogColor[4] = {1.0, 1.0, 1.0, 1.0};
	// Clear screen to background color.
	glClear(GL_COLOR_BUFFER_BIT);

	Question5();

	//handling fog effect
	if (isFog) glEnable(GL_FOG);
	else glDisable(GL_FOG);
	glHint(GL_FOG_HINT, GL_NICEST);
	glFogfv(GL_FOG_COLOR, fogColor);
	glFogi(GL_FOG_MODE, fogMode);
	glFogf(GL_FOG_START, fogStart);
	glFogf(GL_FOG_END, fogEnd);

	//display the wireframe, no color needed
	if (isWire) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); 
	else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//for translating the object
	glTranslatef(unitX, unitY, unitZ);
	//for rotating the object
	glTranslatef(0, 0, -2);
	glRotatef(angleX, 1.0, 0.0, 0.0);
	glRotatef(angleY, 0.0, 1.0, 0.0);
	glRotatef(angleZ, 0.0, 0.0, 1.0);
	glTranslatef(0, 0, 2);

	// Set foreground (or drawing) color.
	glColor3f(0.0, 0.0, 0.0);

	glCallList(aModelList);

	glutSwapBuffers();
}

// Initialization routine.
void setup(char *fileName) 
{
	// Set background (or clearing) color.
	glClearColor(1.0, 1.0, 1.0, 0.0); 

	Question1(fileName);
	Question3();

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, movedVertexArray);

	//as required, generate a display list
	aModelList = glGenLists(1);
	glNewList(aModelList, GL_COMPILE);

	// Draw a polygon with specified vertices.
	int verticeBegIdx = 0;
	for(unsigned int i = 0; i < numVerticesPerFace.size(); i++) {
		if(numVerticesPerFace[i] == 3) {
			glBegin(GL_TRIANGLES);
			glArrayElement(displayList[verticeBegIdx]);
			glArrayElement(displayList[verticeBegIdx+1]);
			glArrayElement(displayList[verticeBegIdx+2]);
			glEnd();
			verticeBegIdx += 3;
		}
		else {
			glBegin(GL_POLYGON);
			glArrayElement(displayList[verticeBegIdx]);
			glArrayElement(displayList[verticeBegIdx+1]);
			glArrayElement(displayList[verticeBegIdx+2]);
			glArrayElement(displayList[verticeBegIdx+3]);
			glEnd();
			verticeBegIdx += 4;
		}
	}
	glEndList();
}

// OpenGL window reshape routine.
void resize(int w, int h)
{
	// Set viewport size to be entire OpenGL window.
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glOrtho(-1.0, 1.0, -1.0, 1.0, 1.0, 100.0);
	// Set matrix mode to modelview.
	glMatrixMode(GL_MODELVIEW);

	// Clear current modelview matrix to identity.
	glLoadIdentity();
}

// Keyboard input processing routine.
void keyInput(unsigned char key, int x, int y)
{
	switch(key) 
	{
	// Press escape to exit.
	case 'q':
		exit(0);
		break;
	case 'w':
		Question2();
		break;
	case 'v':
		orthoOrPers = true;
		break;
	case 'V':
		orthoOrPers = false;
		break;
	case 'l':
		unitX += -0.1;
		break;
	case 'L':
		unitX += 0.1;
		break;
	case 'd':
		unitY += -0.1;
		break;
	case 'D':
		unitY += 0.1;
		break;
	case 'n':
		unitZ += -0.1;
		break;
	case 'N':
		unitZ += 0.1;
		break;
	case 'p':
		angleX += -10;
		break;
	case 'P':
		angleX += 10;
		break;
	case 'y':
		angleY += -10;
		break;
	case 'Y':
		angleY += 10;
		break;
	case 'r':
		angleZ += -10;
		break;
	case 'R':
		angleZ += 10;
		break;
	case 's':
		unitX = 0; unitY = 0; unitZ = 0;
		angleX = 0; angleY = 0; angleZ = 0;
		eyeXO = 0; eyeYO = 0; eyeZO = 0;
		centerXO = 0; centerYO = 0; centerZO = -2.0;
		upXO = 0; upYO = 1; upZO = 0;
		eyeXP = 0; eyeYP = 0; eyeZP = 0.0;
		centerXP = 0; centerYP = 0; centerZP = -2.0;
		upXP = 0; upYP = 1; upZP = 0;
		angleLX = 0; angleLY = 0; angleLZ = 0;
		break;
	case 'f':
		isFog = false;
		break;
	case 'F':
		isFog = true;
		break;
	case 'i':
		eyeZP += -0.1;
		eyeZO += -0.1;
		break;
	case 'I':
		eyeZP += 0.1;
		eyeZO += 0.1;
		break;
	case 'a':
		angleLY += 0.174532925; //the value of 10/180*PI
		centerXP = sin(angleLY);
		centerZP = -cos(angleLY);
		centerXO = sin(angleLY);
		centerZO = -cos(angleLY);
		break;
	case 'A':
		angleLY += -0.174532925;
		centerXP = sin(angleLY);
		centerZP = -cos(angleLY);
		centerXO = sin(angleLY);
		centerZO = -cos(angleLY);
		break;
	case 't':
		angleLX += 0.174532925;
		centerYP = -sin(angleLX);
		centerZP = -cos(angleLX);
		centerYO = -sin(angleLX);
		centerZO = -cos(angleLX);
		upZP = -sin(angleLX);
		upYP = cos(angleLX);
		upZO = -sin(angleLX);
		upYO = cos(angleLX);
		break;
	case 'T':
		angleLX += -0.174532925;
		centerYP = -sin(angleLX);
		centerZP = -cos(angleLX);
		centerYO = -sin(angleLX);
		centerZO = -cos(angleLX);
		upZP = -sin(angleLX);
		upYP = cos(angleLX);
		upZO = -sin(angleLX);
		upYO = cos(angleLX);
		break;
	case 'c':
		angleLZ += 0.174532925;
		upXP = sin(angleLZ);
		upYP = cos(angleLZ);
		upXO = sin(angleLZ);
		upYO = cos(angleLZ);
		break;
	case 'C':
		angleLZ += -0.174532925;
		upXP = sin(angleLZ);
		upYP = cos(angleLZ);
		upXO = sin(angleLZ);
		upYO = cos(angleLZ);
		break;
	default:
		break;
	}
	glutDisplayFunc(drawScene); 
	glutPostRedisplay();
}

void specialKeys(int key, int x, int y)
{
	switch(key) 
	{
	case GLUT_KEY_LEFT:
		eyeXP += -0.1;
		eyeXO += -0.1;
		centerXP += -0.1; //if these two lines are commented, then the
		centerXO += -0.1; //camera is always looking at the object while moving
		break;
	case GLUT_KEY_RIGHT:
		eyeXP += 0.1;
		centerXP += 0.1;
		eyeXO += 0.1;
		centerXO += 0.1;
		break;
	case GLUT_KEY_UP:
		eyeYP += 0.1;
		centerYP += 0.1;
		eyeYO += 0.1;
		centerYO += 0.1;
		break;
	case GLUT_KEY_DOWN:
		eyeYP += -0.1;
		centerYP += -0.1;
		eyeYO += -0.1;
		centerYO += -0.1;
		break;
	default:
		break;
	}
	glutDisplayFunc(drawScene); 
	glutPostRedisplay();
}

// Main routine: defines window properties, creates window,
// registers callback routines and begins processing.
int main(int argc, char **argv) 
{  
	// Initialize GLUT.
	glutInit(&argc, argv);

	// Set display mode as single-buffered and RGB color.
	//glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB); 
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB); 

	// Set OpenGL window size.
	glutInitWindowSize(500, 500);

	// Set position of OpenGL window upper-left corner.
	glutInitWindowPosition(100, 100); 

	// Create OpenGL window with title.
	glutCreateWindow("assign1.cpp");

	// Initialize.
	setup(argv[1]);

	// Register display routine.
	glutDisplayFunc(drawScene); 

	// Register reshape routine.
	glutReshapeFunc(resize);  

	// Register keyboard routine.
	glutKeyboardFunc(keyInput);
	glutSpecialFunc(specialKeys);

	// Begin processing.
	glutMainLoop(); 

	return 0;  
}
