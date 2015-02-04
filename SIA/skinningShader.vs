const int max_bones = 50; // should be the same as in the client code
uniform mat4 matrices[max_bones];
const int nb_influences = 4;
attribute vec4 weightIndex;
attribute vec4 weight;

void main()
{
		vec4 pos = gl_Vertex;
        
		gl_Position = gl_ModelViewProjectionMatrix * pos;
        		
        gl_FrontColor = gl_Color;
        gl_BackColor = gl_Color;

}