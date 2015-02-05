#include "viewer.h"
#include <qfiledialog.h>

using namespace std;

std::string jointNameCol = "lhumerus";

#if _SKINNING_GPU
#define BUFFER_OFFSET(a) ((char*)NULL + (a))
///////////////////////////////////////////////////////////////
//Shader
GLuint MyProgram;									// Shader handle
char* ShaderNoLighting = (char*) "skinningShader";	// Shader file names
char* Shader=(char*) "";
//////////////////////////////////////////////////////////////
// Points
unsigned int num_points;
typedef glm::vec4 Point;
Point *initial_points;
GLuint initial_points_vboid;
//////////////////////////////////////////////////////////////
// Normals
typedef glm::vec3 Normal;
Normal *initial_normals;
GLuint initial_normals_vboid;
//////////////////////////////////////////////////////////////
// Triangles
unsigned num_triangle_indices;
GLuint triangles_ibo;
bool surface_lighting = false;
//////////////////////////////////////////////////////////////
// Bones
unsigned int num_bones;
const unsigned int max_bones = 250;
glm::mat4 m_transfo[max_bones];
const unsigned int m_origin_size = max_bones * sizeof(glm::mat4);
//////////////////////////////////////////////////////////////
// bone_weight
const unsigned num_frames_per_point = 4;
float *bone_weight;
float *bone_index;
GLuint bone_weight_vboid;				/* weight VBO id */
GLint bone_weight_attrib_location;		/* weight id in shader */
GLuint bone_index_vboid;				/* VBO id */
GLint bone_index_attrib_location;		/* id in shader */


template<typename T>
inline void create_buffer_object( GLuint* vbo, unsigned long BUFFER_TYPE, unsigned long ACCESS_TYPE, T* data, unsigned num_variables )
{
    glGenBuffers(1, vbo);
    glBindBuffer(BUFFER_TYPE, *vbo);
    unsigned int vbo_size = num_variables * sizeof(T);
    glBufferData(BUFFER_TYPE, vbo_size, 0, ACCESS_TYPE);
    glBufferSubData(BUFFER_TYPE, 0, vbo_size, data);
    glBindBuffer(BUFFER_TYPE, 0);
}

void ReloadShader()
{
    cerr << "-Compiling-------------------------------------" << endl;

    char FragmentShaderFile[1000];
    char VertexShaderFile[1000];

    Shader = ShaderNoLighting;

    sprintf_s( FragmentShaderFile, "%s.fs", Shader );
    sprintf_s( VertexShaderFile, "%s.vs", Shader );

    GLchar* SourceCode = new GLchar[10000];
    FILE* SourceFile;
    GLint SourceCodeLen;
    GLuint MyFragmentShader;
    GLuint MyVertexShader;
    GLchar Log[10000];
    int L;

    //Read fragment shader source
	fopen_s( &SourceFile, FragmentShaderFile, "r" );
    if( !SourceFile )
    {
        cerr << "Could not open " << FragmentShaderFile << " for reading" << endl;
        return;
    }
    SourceCodeLen = (GLint)fread( SourceCode, 1, 10000, SourceFile );
    fclose( SourceFile );

    MyFragmentShader = glCreateShader( GL_FRAGMENT_SHADER );
    const char *FragmentLines = SourceCode;
    glShaderSource( MyFragmentShader, 1, &FragmentLines, &SourceCodeLen );
    //Compile fragment shader
    glCompileShader( MyFragmentShader );
    glGetShaderInfoLog( MyFragmentShader, 10000, &L, Log );
    if( L ) cerr << "Compile log for fragment shader : " << endl << Log << endl;
    else cerr << "Fragment shader Ok" << endl;


    //Read vertex shader source
    fopen_s(&SourceFile, VertexShaderFile, "r" );
    if( !SourceFile )
    {
        cerr << "Could not open " << VertexShaderFile << " for reading" << endl;
        return;
    }
    SourceCodeLen = (GLint)fread( SourceCode, 1, 10000, SourceFile );
    fclose( SourceFile );

    MyVertexShader = glCreateShader( GL_VERTEX_SHADER );
    const char *VertexLines = SourceCode;
    glShaderSource( MyVertexShader, 1, &VertexLines, &SourceCodeLen );
    //Compile vertex shader
    glCompileShader( MyVertexShader );
    glGetShaderInfoLog( MyVertexShader, 10000, &L, Log );
    if( L ) cerr << "Compile log for vertex shader : " << endl << Log << endl;
    else cerr << "Vertex shader Ok" << endl;


    //Link fragment and vertex shaders
    MyProgram = glCreateProgram();
    glAttachShader( MyProgram, MyFragmentShader );
    glAttachShader( MyProgram, MyVertexShader );
    glLinkProgram( MyProgram );
    glGetProgramInfoLog( MyProgram, 10000, &L, Log );
    if( L ) cerr << "Compile log for link : " << endl << Log << endl;
    else cerr << "Link Ok" << endl;
    cerr << flush;
}
#endif

// Draws scene
void Viewer::draw()
{
	glPushMatrix();
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
		glScalef(0.05f, 0.05f, 0.05f);
		glRotatef(90, 1, 0, 0);
		// draw animation skeleton :
		if (_root) _root->draw();
		// draw mesh :
#if _SKINNING_GPU
		if (_human) {
			for (int i = 0 ; i < _skinning->_nbJoints ; i++) {
				m_transfo[i] = _skinning->_transfoCurr[i]*_skinning->_transfoInitInv[i];
			}
			// Activate the shader
			glUseProgram( MyProgram );
			GLint shader_matrices = glGetUniformLocation( MyProgram, "matrices" );
			assert( shader_matrices != -1 );
			glUniformMatrix4fv( shader_matrices, num_bones, false, glm::value_ptr(m_transfo[0]));

			// activate weight data
			glBindBuffer(GL_ARRAY_BUFFER, bone_weight_vboid);
			glVertexAttribPointer(bone_weight_attrib_location, num_frames_per_point, GL_FLOAT, GL_FALSE, 0, 0 );
			glEnableVertexAttribArray(bone_weight_attrib_location);

			// activate weight index data
			glBindBuffer(GL_ARRAY_BUFFER, bone_index_vboid);
			glVertexAttribPointer(bone_index_attrib_location, num_frames_per_point, GL_FLOAT, GL_FALSE, 0, 0 );
			glEnableVertexAttribArray(bone_index_attrib_location);

			// activate vertex data
			glBindBuffer(GL_ARRAY_BUFFER, initial_points_vboid);
			glVertexPointer(4, GL_FLOAT, 0, BUFFER_OFFSET(0));
			glEnableClientState(GL_VERTEX_ARRAY);

			glBindBuffer(GL_ARRAY_BUFFER, initial_normals_vboid);
			glNormalPointer(GL_FLOAT, 0, 0);
			glEnableClientState(GL_NORMAL_ARRAY);

			// draw
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triangles_ibo);
			glColor4f(_human->_color.x,_human->_color.y,_human->_color.z,_human->_color.w);
			glDrawElements(GL_TRIANGLES, num_triangle_indices, GL_UNSIGNED_INT, 0 );
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		
			// deactivate data
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_NORMAL_ARRAY);
			glDisableVertexAttribArray(bone_weight_attrib_location);
			glDisableVertexAttribArray(bone_index_attrib_location);
			glUseProgram( 0 );
		}
#else
		if (_human) _human->draw();
#endif
		glDisable(GL_BLEND);
	}
	glPopMatrix();
}

void Viewer::animate() 
{
	// Update frame number :
	_iframe = (_iframe+1)%_nframes;

	// Animate skeleton :
	if (_root) _root->animate(_iframe);

	// Apply skinning to mesh :
	if (_skinning) _skinning->animate();
}

void Viewer::init()
{
  // Restore previous viewer state.
  restoreStateFromFile();
  setBackgroundColor(QColor(255, 255, 255));

  // set new key descriptions :
  setKeyDescription(Qt::Key_L, "Load an animation (.bvh)");
  setKeyDescription(Qt::Key_W, "Change weight computation method");

  // Load skeleton :
  _root = NULL;
  _root = Skeleton::createFromFile("data/walk.bvh");
  cout << "MinY : " << _root->getMotionBegin() << endl;
  cout << "Period : " << _root->getMotionPeriod() << endl;
  if (_root->_dofs.size())
	  _nframes = _root->_dofs[0]._values.size();
  else
	  _nframes = 0;
  _iframe = 0;
  _root->nbDofs();

  _human = NULL;
  _skinning = NULL;

#if _SKINNING_ON
  // Load mesh :
  _human = new Mesh();
  _human->load("data/human9.obj");

  // Init skinning :
  _skinning = new Skinning();
  _skinning->_skin = _human;
  _skinning->_skel = _root;
  _skinning->init();
  _skinning->paintWeights(jointNameCol);

#if _SKINNING_GPU
  glewInit();
  // Load shaders
  ReloadShader();
  // Get mesh info to shader :
  num_points = _human->_points.size();
  initial_points = (Point*)calloc(sizeof(Point), num_points);
  initial_normals = (Normal*)calloc(sizeof(Normal), num_points);
  for (unsigned int i = 0 ; i < num_points ; i++) {
	  initial_points[i] = _skinning->_pointsInit[i];
	  initial_normals[i] = _human->_normals[i];
  }
  vector<unsigned int> triangles;
  for (unsigned int i = 0 ; i < _human->_triangles.size() ; i++) {
	  triangles.push_back(_human->_triangles[i]);
  }
  // Get bone info to shader :
  num_bones = _skinning->_nbJoints;
  // Get skinning info to shader :
  bone_weight = (float*)calloc(sizeof(float),num_points * num_frames_per_point);
  bone_index = (float*)calloc(sizeof(float),num_points * num_frames_per_point);
  printf("%d x %d = %d\n", num_points, num_frames_per_point, num_points * num_frames_per_point);
  int k = 0;
  for (unsigned int i = 0 ; i < num_points ; i++) {
	  int k1 = 0;
	  for (unsigned int j = 0 ; j < num_bones ; j++) {
		  if (!_skinning->_weights[i][j]) continue;
		  bone_weight[k] = (float)_skinning->_weights[i][j];
		  bone_index[k] = j;
		  k++;
		  k1++;
	  }
	  for (int j = k1 ; j < num_frames_per_point ; j++) {
		  bone_weight[k] = (float)0;
		  bone_index[k] = 0;
		  k++;
	  }
  }
  
  // create buffer objects
  create_buffer_object( &initial_points_vboid , GL_ARRAY_BUFFER, GL_STATIC_DRAW, initial_points , num_points );
  create_buffer_object( &initial_normals_vboid, GL_ARRAY_BUFFER, GL_STATIC_DRAW, initial_normals, num_points );
  create_buffer_object( &bone_weight_vboid, GL_ARRAY_BUFFER, GL_STATIC_DRAW, bone_weight, num_points * num_frames_per_point );
  create_buffer_object( &bone_index_vboid, GL_ARRAY_BUFFER, GL_STATIC_DRAW, bone_index, num_points * num_frames_per_point );

  num_triangle_indices = triangles.size();
  create_buffer_object( &triangles_ibo, GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW, &triangles[0], triangles.size()  );
  cerr << triangles.size()/3 << " triangles" << endl;

  /** Get access to shader data */
  glUseProgram( MyProgram );
  bone_weight_attrib_location = glGetAttribLocation(MyProgram, "weight");
  //assert (bone_weight_attrib_location!=-1 ); 
  bone_index_attrib_location = glGetAttribLocation(MyProgram, "weightIndex");
  //assert (bone_index_attrib_location!=-1 );
  glUseProgram( 0 );
#endif
#endif
  
  // Opens help window
  //help();
}

void Viewer::keyPressEvent(QKeyEvent *e)
{
	QString filename;
	char fname[600];
	switch (e->key()) {
		case Qt::Key_L :		// Load new mocap sequence
			if (_root) delete _root;
			_root = NULL;
			filename = QFileDialog::getOpenFileName(this, tr("Select a mocap sequence"), "data/", tr("Mocap (*.bvh);;All files (*)"));
			sprintf_s(fname, "%s", filename.toAscii().data());
			_root = Skeleton::createFromFile(fname);
			if (_root->_dofs.size())
				_nframes = _root->_dofs[0]._values.size();
			else
				_nframes = 0;
			_iframe = 0;
			_root->nbDofs();
			_skinning->_skel = _root;
			_skinning->recomputeWeights();
			if (_skinning->_skin->_colors.size()) {
				cout << "paint weights" << endl;
				_skinning->paintWeights(jointNameCol);
			}
			animate();
			updateGL();
			break;
		case Qt::Key_W :		// Modify computation of weights for skinning
			if (!_skinning) return;
			_skinning->_meth = (_skinning->_meth+1)%3;
			_skinning->recomputeWeights();
			if (_skinning->_skin->_colors.size()) {
				cout << "paint weights" << endl;
				_skinning->paintWeights(jointNameCol);
			}
			updateGL();
			break;
		case Qt::Key_Escape :	// quit
			saveStateToFile();
			exit(1);
			break;
		case Qt::Key_Q :		// quit
			saveStateToFile();
			exit(1);
			break;
		default :
			QGLViewer::keyPressEvent(e);
			break;
	}
}

QString Viewer::helpString() const
{
  QString text("<h2>c h a r a c t e r A n i m</h2>");
  text += "Use the mouse to move the camera around the object. ";
  text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
  text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
  text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
  text += "Simply press the function key again to restore it. Several keyFrames define a ";
  text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
  text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
  text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
  text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
  text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
  text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
  text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
  text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
  text += "Press <b>Escape</b> to exit the viewer.";
  return text;
}
