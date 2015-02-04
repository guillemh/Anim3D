/*************************************************************
GLSL Demo - Sandbox
(c) 2005 Antoine Bouthors, EVASION
*************************************************************/

varying vec3 OutColor;

void main()
{
       gl_FragColor = gl_Color;  // interpolated from gl_FrontColor and gl_BackColor
}
