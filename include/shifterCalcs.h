#define analogOutputPin A16         // Digital Pin 40

unsigned int shiftRatio;  
const float gear1 = 0.002490845;     
const float gear2 = 0.003216934;
const float gear3 = 0.003859545;
const float gear4 = 0.004455593;
const float gear5 = 0.005003002;
const float gear6 = 0.005594663;   

const float t12 = (gear2-gear1)/2;
const float t23 = (gear2-gear3)/2;
const float t34 = (gear3-gear4)/2;
const float t45 = (gear4-gear5)/2;
const float t56 = (gear5-gear6)/2;

void checkShiftRatio();