// flowIO.h

// the "official" threshold - if the absolute value of either 
// flow component is greater, it's considered unknown
#define UNKNOWN_FLOW_THRESH 1e9

// value to use to represent unknown flow
#define UNKNOWN_FLOW 1e10

// return whether flow vector is unknown
bool unknown_flow(float u, float v);
bool unknown_flow(float *f);

// read a flow file into 2-band image
void ReadFlowFile(CFloatImage& img, const char* filename);

// write a 2-band image into flow file 
void WriteFlowFile(CFloatImage img, const char* filename);


