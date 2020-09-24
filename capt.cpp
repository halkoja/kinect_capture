#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

void writeDepth(const char*, libfreenect2::Frame*);
void writeDepth_bin(const char *fname, libfreenect2::Frame *frame);

void writePC(const  char *fname, std::vector<std::vector<float>>);
void writePC_ply(const  char *fname, std::vector<std::vector<float>>);

std::vector<std::vector<float>> createPointCloud(libfreenect2::Frame*, libfreenect2::Registration*, int minDist, int maxDist);

// TODO: 
// -createPointCloud, add RGB?
// -Average depth matrices over multiple frames
int main(int argc, char* argv[]) {
  // Discover device(s)
  int nDev;
  libfreenect2::Freenect2 freenect2;
  nDev = freenect2.enumerateDevices();
  std::cout << "Found " << nDev << " device(s).\n";

  // Use OpenGL for processing
  libfreenect2::PacketPipeline *pipeline = 0;
  pipeline = new libfreenect2::OpenGLPacketPipeline();
  
  // Open device
  libfreenect2::Freenect2Device *dev = 0;
  dev = freenect2.openDefaultDevice(pipeline);

  if(dev == 0) {
    std::cout << "Failure.\n";
    return EXIT_FAILURE;
  }
  
  // Frame & listener
  int type = libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener(type);
  libfreenect2::FrameMap frames;

  libfreenect2::Frame acc(512, 424, 4);
  float* accp = (float*) acc.data, *dpointer;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);

  // Start streaming
  bool enable_rgb = false, enable_depth = true;
  if (!dev->startStreams(enable_rgb, enable_depth))
    return EXIT_FAILURE;


  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;


  int counter = 0;
  int angle = (argc>2?atol(argv[2]):360);
  int cMax = 360/angle;

  int w = acc.width, h = acc.height;

  char terminate = 0;
  
  bool write_files = false;

  std::string fname;
  if(argc > 1) {
    fname = argv[1];
    write_files = true;
  }

  int n,k;

  int minDist = 700, maxDist = 900;

  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

  while (terminate ~= 'q' && counter < cMax) {
    for (k=0; k<w*h; k++)
      accp[k] = 0;

    if (!listener.waitForNewFrame(frames, 5*1000)) // 5 seconds
      {
	std::cout << "timeout!" << std::endl;
	return EXIT_FAILURE;
      }

    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
      
    // If we don't fetch rgb and ir data, the program will freeze.
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    dpointer = (float*) depth->data;
    for (k = 0; k < w*h; k++)
      accp[k] = dpointer[k];
    // Relevant data has been written, release frames.
    listener.release(frames);

    if(write_files) {
      std::string num = std::to_string(++counter);
      std::string fname_format = fname + std::string(3-num.length(),'0') + num;
    
      writeDepth_bin(fname_format.c_str(), &acc);
      std::cout << "\nWrote file \"" << fname_format << "\".\n\n";
      
      std::string pcname = "pc_" + fname_format;
      std::vector<std::vector<float>> pc = createPointCloud(&acc,registration, minDist, maxDist);
      writePC_ply(pcname.c_str(), pc);
    }

    if(counter < cMax) {
      std::cout << "Continue? (q to quit)\n";
      std::cin.get(terminate);
    }
  }
  dev->stop();
  dev->close();

  return EXIT_SUCCESS;
}

void writeDepth(const char *fname, libfreenect2::Frame *frame) {
  std::ofstream file;
  file.open(fname, std::ios::out);

  int w = frame->width, h = frame->height;
  float *data = (float*) frame->data;

  int j,k;
  for(j = 0; j < h; j++) {
    for(k = 0; k < w; k++)
      file << data[k + j*w] << (k+1<w?',':'\n');
  }
  
  file.close();
  
  return;
}

std::vector<std::vector<float>> createPointCloud(libfreenect2::Frame* depth, libfreenect2::Registration* registration, int minDist, int maxDist) {
  std::vector<std::vector<float>> pc;

  return pc;
}

void writePC(const  char *fname, std::vector<std::vector<float>> pc) {
  std::ofstream file;
  file.open(fname, std::ios::out);

  for (std::vector<std::vector<float>>::iterator vec = pc.begin(); vec != pc.end(); vec++)
    file << (*vec)[0] << ',' << (*vec)[1] << ',' << (*vec)[2] << std::endl;

  file.close();

  return;
}

void writePC_ply(const char *fname, std::vector<std::vector<float>> pc) {
  std::ofstream file;
  file.open(fname, std::ios::binary);
  std::string header = 
    "ply\n"
    "format binary_little_endian 1.0\n"  
    "comment pointcloud from kinect\n" 
    "element vertex " +
    std::to_string(pc.size()) +
    "\nproperty float x\n" 
    "property float y\n" 
    "property float z\n" 
    "end_header\n";

  file << header;

  std::vector<float> temp = {0,0,0};

  int j,k;
  for (j = 0; j < pc.size(); j++)
    for (k = 0; k < 3; k++) {
      temp = pc[j];
      file.write((char*) &(temp[k]), sizeof(float));
    }
  
  file.close();
}

void writeDepth_bin(const char *fname, libfreenect2::Frame *frame) {
  std::ofstream file;
  file.open(fname, std::ios::binary);

  int w = frame->width, h = frame->height;
  std::string header = std::to_string(w) + ' ' + std::to_string(h) + '\n';
  file << header;

  char* data = (char*) frame->data;
  file.write( data, h*w*sizeof(float));

  file.close();
  return;
}
