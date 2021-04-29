
#ifdef  WIN32
	#include <windows.h>
#endif
#include <getopt.h>
#include <iostream>
#include <GL/glew.h>
#include <GL/glut.h>
#include <vector>
#include <string>

#include <experimental/filesystem>

#include "ReactionTestInterface.h"

int main(int argc, char ** argv)
{
	std::cout<<"S : Play"<<std::endl;
	std::cout<<"ESC : exit"<<std::endl;

	std::string ppo="", bvh="", reg="";
	static struct option long_options[] =
    {
		{"bvh", required_argument,        NULL, '1'},
		{"ppo", required_argument,        NULL, '2'},
    }; 
  	int option_index = 0;
  	while (1)
    {
  		auto c = getopt_long(argc, argv, "12:",
                       long_options, &option_index);
      	if (c == -1)
        	break;
      	switch (c)
      	{
	           case 0:
	             break;

	           case '1':
	             bvh= optarg;
	             break;

	           case '2':
	              ppo = optarg;
	   			  break; 
       	}
    }

	glutInit(&argc, argv);	
	ReactionTestInterface* interface = new ReactionTestInterface(bvh, ppo);
    interface->GLInitWindow("Motion Control");
	glutMainLoop();
	return 0;
}
