#include "VisualTracker.h"

int main(void)
{
   VisualTracker vt;
   
   std::vector<int> XY(2);
   XY[0] = 0;
   XY[1] = 0;

   for(int i=0; i<10; i++)
   {
      vt.get_realRobot_XY(XY);
      std::cout << std::endl << "X= " << XY[0] << "    Y= " << XY[1] << std::endl;
   }

   return 0;
}
