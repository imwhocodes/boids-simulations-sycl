#include <CL/sycl.hpp>
#include <iostream>
#include <chrono>
#include <memory>
#include <vector>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "json.hpp"
#include "GLWindow.hpp"
#include "Boid.hpp"
#include "Queue.hpp"


namespace sycl =  cl::sycl;

using Clock_t = std::chrono::high_resolution_clock;


constexpr size_t BOIDS_NUM = 6'000;//45'000;//20'000;//3000000;
constexpr size_t MULTIPLE_SIM = 4;


constexpr std::chrono::nanoseconds FPS_PERIOD_TARGET = std::chrono::nanoseconds(int(1.0E9 / 60.0)) ;


using CT = SimulationParams::Coord_t;
using ET = CT::element_type;

float_t movement(const float_t period, const size_t counter){

  const float_t alpha = counter / period;

  return (std::cos(alpha * M_PI * 2.0) + 1.0) / 2.0;
}


template<typename T>
T movementLerp(const T & min, const T & max, const float_t period, const size_t counter){

  return sycl::mix(min, max, movement(period, counter));
}



SimulationParams::Ptr generateParam(const size_t counter){




  //FROM LIVE PARAM
  return std::make_shared<SimulationParams>(
    CT{ ET{ 15 } }, //SIM Size
    movementLerp( CT{0.20, 0.35, 0.55}, CT{1.2, 1.7, 2}, 6000, counter),    //Radiuses
    movementLerp( CT{0.5, 0.5, 0.5}, CT{2, 2, 2}, 15000, counter),  // CT{1, 1, 1},             //Weights
    CT{INFINITY, 120, 180},  //Angles
    0.20,
    0.025,
    0.15
  );

  // return sim_params_ptr;

  // using CT = SimulationParams::Coord_t;
  // using ET = CT::element_type;

  // return std::make_shared<SimulationParams>(
  //           CT{ ET( movementLerp(3.0, 7.0, 34'000, counter) ) }, //SIM Size

  //           movementLerp( CT{ 0.20, 0.35, 0.55 }, CT{ 0.35, 0.65, 0.90 }, 7'553, counter),  //Radiuses

  //           CT{ 1, 1, 1},             //Weights

  //           movementLerp( CT{ INFINITY, 120, 180}, CT{ INFINITY, 180, 90}, 12'221, counter),  //Angles

  //           movementLerp(0.05, 0.15, 4'556, counter),

  //           0.0075,
  //           0.15
  //         );
}





using MultiSimSamples_t = std::array<SimSample::Ptr,        MULTIPLE_SIM>;
using MultiParam_t =      std::array<SimulationParams::Ptr, MULTIPLE_SIM>;


Queue<MultiSimSamples_t>  sample_queue;
Queue<MultiParam_t>       param_queue;

GLFWwindow * GL_window;


void readParams(){

  using json = nlohmann::json;

  constexpr size_t MAX_SIZE = 1500;


  int socket_desc;
  struct sockaddr_in server_addr;


  socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(55555);
  server_addr.sin_addr.s_addr = inet_addr("0.0.0.0");

  bind(socket_desc, (struct sockaddr*)&server_addr, sizeof(server_addr));

  while(true){

    // std::string input;

    // std::getline(std::cin, input);

    // std::istringstream iss(input);

    std::array<char, MAX_SIZE> recive_buffer;

    const size_t recived_len = recv(socket_desc, recive_buffer.data(), MAX_SIZE, 0);

    std::string_view input{recive_buffer.data(), recived_len};

    // std::cout << "Recived:\t" << input << std::endl;

    try{

      const json j = json::parse(input);

      std::shared_ptr<MultiParam_t> multi_sim_params_ptr = std::make_shared<MultiParam_t>();

      for(size_t i = 0; i < multi_sim_params_ptr->size(); i++){

        const json  p = j[std::to_string(i)];

        const json size       = p["size"];
        const json radiuses   = p["radiuses"];
        const json weights    = p["weights"];
        const json angles     = p["angles"];
        const json max_speed  = p["max_speed"];
        const json accel      = p["accel"];
        const json repel      = p["repel"];


        (*multi_sim_params_ptr)[i] = std::make_shared<SimulationParams>(
                                  CT{ ET{size[0]},      ET{size[1]},      ET{size[2]}     },  //SIM Size
                                  CT{ ET{radiuses[0]},  ET{radiuses[1]},  ET{radiuses[2]} },  //Radiuses
                                  CT{ ET{weights[0]},   ET{weights[1]},   ET{weights[2]}  },  //Weights
                                  CT{ ET{angles[0]},    ET{angles[1]},    ET{angles[2]}   },  //Angles
                                  max_speed,  //0.20,
                                  accel,      //0.025,
                                  repel       //0.15
                              );
      }

      param_queue.pushOverwite(multi_sim_params_ptr);

    }

    catch(const json::exception & e){
      std::cerr << e.what() << "\t:\t" << input << std::endl;
    }

    // for( float_t & f : input_values){
    //   std::string value;

    //   std::getline(iss, value, ';');

    //   f = std::stof(value);

    // }


    // const float_t mult = input_values[0];

    // std::cout << "Readed:\t" << mult << "\t:\t" << mult << std::endl;

    //   //FROM LIVE PARAM
    // SimulationParams::Ptr sim_params_ptr = std::make_shared<SimulationParams>(
    //   SimulationParams::Coord_t{SimulationParams::Coord_t{SimulationParams::Coord_t::element_type{7}}}, //SIM Size
    //   sycl::mix(CT{0.20, 0.35, 0.55},  CT{1, 0.50, 1.2}, mult),//Radiuses
    //   // SimulationParams::Coord_t{0.20, 0.35, 0.55},    //Radiuses
    //   CT{1.0, 1.0, 1.0},// sycl::mix(CT{1.0, 1.0, 1.0}, CT{0.15, 1.5, 0.6}, mult), //SimulationParams::Coord_t{1, 1, 1},             //Weights
    //   SimulationParams::Coord_t{INFINITY, 120, 180},  //Angles
    //   0.05,
    //   0.0075,
    //   0.15
    // );


  }

}

void nextGen(){

  // sycl::gpu_selector device_selector;
  sycl::default_selector device_selector;
  sycl::queue gpu_queue(device_selector);

  // sycl::queue gpu_queue(device_selector);
  // std::cout << "Running on " << gpu_queue.get_device().get_info<sycl::info::device::name>() << "\n";


  auto last_frame_ticks_real = Clock_t::now();


  //FROM LIVE PARAM
  // SimulationParams::Ptr sim_params_ptr = std::make_shared<SimulationParams>(
  //   SimulationParams::Coord_t{SimulationParams::Coord_t{SimulationParams::Coord_t::element_type{7}}}, //SIM Size
  //   SimulationParams::Coord_t{0.20, 0.35, 0.55},    //Radiuses
  //   SimulationParams::Coord_t{1, 1, 1},             //Weights
  //   SimulationParams::Coord_t{INFINITY, 120, 180},  //Angles
  //   0.05,
  //   0.0075,
  //   0.15
  // );


  size_t counter = 0;

  const SimulationParams::Ptr sim_params_ptr_init = generateParam(counter);

  std::shared_ptr<MultiSimSamples_t> last_simulations_boids = std::make_shared<MultiSimSamples_t>();

  // // for(SimSample::Ptr & s : *last_simulations_boids){

  // //   s = 

  // // }


   (*last_simulations_boids)[0] = std::make_shared<SimSample>(BoidBase::RandomVect(BOIDS_NUM, *sim_params_ptr_init), generateParam(1000));
   (*last_simulations_boids)[1] = std::make_shared<SimSample>(BoidBase::RandomVect(BOIDS_NUM, *sim_params_ptr_init), generateParam(50000));
   (*last_simulations_boids)[2] = std::make_shared<SimSample>(BoidBase::RandomVect(BOIDS_NUM, *sim_params_ptr_init), generateParam(10000));
   (*last_simulations_boids)[3] = std::make_shared<SimSample>(BoidBase::RandomVect(BOIDS_NUM, *sim_params_ptr_init), generateParam(475090));


  param_queue.pushOverwite(std::make_shared<MultiParam_t>(MultiParam_t{sim_params_ptr_init, sim_params_ptr_init, sim_params_ptr_init, sim_params_ptr_init}));



  while (!glfwWindowShouldClose(GL_window)){

    const auto start_computing = Clock_t::now();

    const std::shared_ptr<MultiParam_t> sim_params_ptr_ext = param_queue.popGet(); //generateParam(counter); //param_queue.popGet();

    std::shared_ptr<MultiSimSamples_t> last_simulations_boids_updated = std::make_shared<MultiSimSamples_t>();

    for(size_t i = 0; i < MULTIPLE_SIM; i++){
      (*last_simulations_boids_updated)[i] = (*last_simulations_boids)[i]->computeGetNextGenNewParam((*sim_params_ptr_ext)[i], gpu_queue); //->computeGetNextGenNewParam(sim_params_ptr_ext, gpu_queue);
    }

    gpu_queue.wait();

    // gpu_queue.submit([&] (sycl::handler& cgh) {

    //   cgh.parallel_for<class test>( sycl::range<1>(MULTIPLE_SIM), [&](sycl::id<1> ic){
    //     const size_t i = ic[0];
    //     last_simulations_boids_updated[i] = last_simulations_boids[i]->computeGetNextGenNewParam(sim_params_ptr_ext, gpu_queue);
    //   });

    // });

    // BoidBase::VectorPrt_t boids_out = BoidBase::ComputeNextGen(last_simulation_boids, sim_params_ptr_ext, gpu_queue);

    const auto end_computing = Clock_t::now();

    const auto computing_delta_ticks = end_computing - start_computing;


    sample_queue.pushWait(last_simulations_boids_updated);


    last_simulations_boids = last_simulations_boids_updated;

    counter++;

    const auto now = Clock_t::now();

    const auto frame_delta_ticks = now - last_frame_ticks_real;

    last_frame_ticks_real = now;


    // std::cout <<  "Compu max/real fps:\t" <<
    //                                     1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(computing_delta_ticks).count() <<
    //               "  /  " <<
    //                                     1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(frame_delta_ticks).count() <<
    //               '\n' <<
    // std::endl;

  }

}


glm::mat4 rotateMat(const glm::mat4 & in, const float_t d, const sycl::float3 & p){

  glm::mat4 res = in;

  res = glm::rotate(res, d, glm::vec3(1, 0, 0));
  res = glm::rotate(res, d, glm::vec3(0, 1, 0));
  res = glm::rotate(res, d, glm::vec3(0, 0, 1));

  return res;
}






void showBoids(){

  auto last_frame_ticks_real = Clock_t::now();

  auto next_frame_ticks_virtual = last_frame_ticks_real;


  SimulationParams::SetupGL();

  size_t cnt = 0;

  std::array<glm::mat4, 4> rotation_matrixes;


  while(!glfwWindowShouldClose(GL_window)){

    const auto s = sample_queue.popWait();

    const auto draw_start_ticks = Clock_t::now();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers

    const size_t k = cnt / 500;

    glViewport(0, 0, MONITOR_W*0.5, MONITOR_H*0.5); //LB
    glRotatef(0.13, 1, 0, 0);
    glRotatef(0.17, 0, 1, 0);
    glRotatef(0.09, 0, 0, 1);
    (*s)[(0 + k) % MULTIPLE_SIM]->fastDraw(-k);
    // s->params->setupPush(cnt);
    // BoidBase::FastDraw(*(s->boids), *(s->params), 1);
    // s->params->setupPop();


    glViewport(MONITOR_W*0.5, 0, MONITOR_W*0.5, MONITOR_H*0.5); //RB
    (*s)[(1 + k) % MULTIPLE_SIM]->fastDraw(k);
    // s->params->setupPush(cnt * -1.1);
    // BoidBase::FastDraw(*(s->boids), *(s->params), 1);
    // s->params->setupPop();

    glViewport(0, MONITOR_H*0.5, MONITOR_W*0.5, MONITOR_H*0.5); //LT
    (*s)[(2 + k) % MULTIPLE_SIM]->fastDraw(-k*1.5);
    // s->params->setupPush(cnt * +1.25);
    // BoidBase::FastDraw(*(s->boids), *(s->params), 1);
    // s->params->setupPop();


    glViewport(MONITOR_W*0.5, MONITOR_H*0.5, MONITOR_W*0.5, MONITOR_H*0.5); //RT
    (*s)[(3 + k) % MULTIPLE_SIM]->fastDraw(k);
    // s->params->setupPush(cnt * -1.33);
    // BoidBase::FastDraw(*(s->boids), *(s->params), 1);
    // s->params->setupPop();



    // for(const BoidBase & b : *(s->boids)){
    //   b.drawSelf(*(s->params));
    // }

    // glRotatef(0.13, 1, 0, 0);
    // glRotatef(0.17, 0, 1, 0);
    // glRotatef(0.09 , 0, 0, 1);


    const auto draw_end_ticks = Clock_t::now();

    const auto draw_delta_ticks = draw_end_ticks - draw_start_ticks;


    /* Poll for and process events */
    glfwPollEvents();

    const auto e = glGetError();

    if(e){
        std::cout << glErrorToString(e) << std::endl;
    }


    next_frame_ticks_virtual += FPS_PERIOD_TARGET;

    const auto check_sleep = Clock_t::now();

    if(check_sleep < next_frame_ticks_virtual) {

      std::this_thread::sleep_for( next_frame_ticks_virtual - check_sleep);

      while(Clock_t::now() < next_frame_ticks_virtual);
    }

    /* Swap front and back buffers */
    glfwSwapBuffers(GL_window);

    const auto now = Clock_t::now();

    const auto frame_delta_ticks = now - last_frame_ticks_real;

    last_frame_ticks_real = now;

    // std::cout <<
    //             "Draws max/real fps:\t" <<
    //                                           1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(draw_delta_ticks).count()  <<
    //             "   /  " <<
    //                                           1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(frame_delta_ticks).count() <<
    //             '\n' <<
    //             // *(s->params) <<
    //             // '\n' <<
    // std::endl;

    cnt++;

  }

}


int main(int argc, char *argv[]){



  GL_window = CreateOpenWindow(
                                MONITOR_W,
                                MONITOR_H
                            );


  std::thread read_param(readParams);

  std::thread gen_computer(nextGen);

  showBoids();

  std::cout << "!!!END!!!" << std::endl;

}