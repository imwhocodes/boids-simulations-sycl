#include <CL/sycl.hpp>
#include <iostream>
#include <chrono>
#include <memory>
#include <vector>
#include <thread>
#include "GLWindow.hpp"
#include "Boid.hpp"
#include "Queue.hpp"

namespace sycl =  cl::sycl;

using Clock_t = std::chrono::high_resolution_clock;


float_t movement(const float_t period, const size_t counter){

  const float_t alpha = counter / period;

  return (std::cos(alpha * M_PI) + 1.0) / 2.0;
}


template<typename T>
T movementLerp(const T & min, const T & max, const float_t period, const size_t counter){

  return sycl::mix(min, max, movement(period, counter));
}



SimulationParams::Ptr generateParam(const size_t counter){

  //FROM LIVE PARAM
  static SimulationParams::Ptr sim_params_ptr = std::make_shared<SimulationParams>(
    SimulationParams::Coord_t{SimulationParams::Coord_t{SimulationParams::Coord_t::element_type{7}}}, //SIM Size
    SimulationParams::Coord_t{0.20, 0.35, 0.55},    //Radiuses
    SimulationParams::Coord_t{1, 1, 1},             //Weights
    SimulationParams::Coord_t{INFINITY, 120, 180},  //Angles
    0.05,
    0.0075,
    0.15
  );

  return sim_params_ptr;

  using CT = SimulationParams::Coord_t;
  using ET = CT::element_type;

  return std::make_shared<SimulationParams>(
            CT{ ET( movementLerp(3.0, 7.0, 34'000, counter) ) }, //SIM Size

            movementLerp( CT{ 0.20, 0.35, 0.55 }, CT{ 0.35, 0.65, 0.90 }, 7'553, counter),  //Radiuses

            CT{ 1, 1, 1},             //Weights

            movementLerp( CT{ INFINITY, 120, 180}, CT{ INFINITY, 180, 90}, 12'221, counter),  //Angles

            movementLerp(0.05, 0.15, 4'556, counter),

            0.0075,
            0.15
          );
}




constexpr size_t BOIDS_NUM = 10'000;//45'000;//20'000;//3000000;

constexpr std::chrono::nanoseconds FPS_PERIOD_TARGET = std::chrono::nanoseconds(int(1.0E9 / 60.0)) ;



Queue<SimSample> sample_queue;
GLFWwindow * GL_window;


void nextGen(){

  // sycl::gpu_selector device_selector;
  sycl::default_selector device_selector;
  sycl::queue gpu_queue(device_selector);
  std::cout << "Running on " << gpu_queue.get_device().get_info<sycl::info::device::name>() << "\n";


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

  SimulationParams::Ptr sim_params_ptr = generateParam(counter);

  BoidBase::VectorPrt_t last_simulation_boids  = std::make_shared<BoidBase::Vector_t>( BOIDS_NUM );

  for(BoidBase & b : *last_simulation_boids){
    b = BoidBase::Random(*sim_params_ptr);
  } 



  while (!glfwWindowShouldClose(GL_window)){

    const auto start_computing = Clock_t::now();

    BoidBase::VectorPrt_t boids_out = BoidBase::ComputeNextGen(last_simulation_boids, sim_params_ptr, gpu_queue);

    const auto end_computing = Clock_t::now();

    const auto computing_delta_ticks = end_computing - start_computing;


    sample_queue.pushWait(std::make_shared<SimSample>(boids_out, sim_params_ptr));


    last_simulation_boids = boids_out;
    sim_params_ptr = generateParam(counter);
    counter++;

    const auto now = Clock_t::now();

    const auto frame_delta_ticks = now - last_frame_ticks_real;

    last_frame_ticks_real = now;

    std::cout <<  "Compu max/real fps:\t" << 
                                        1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(computing_delta_ticks).count() <<
                  "  /  " <<
                                        1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(frame_delta_ticks).count() << 
                  '\n' <<
    std::endl;

  }

}

void showBoids(){

  auto last_frame_ticks_real = Clock_t::now();

  auto next_frame_ticks_virtual = last_frame_ticks_real;


  while(!glfwWindowShouldClose(GL_window)){

    const auto s = sample_queue.popWait();

    const auto draw_start_ticks = Clock_t::now();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers

    BoidBase::FastDraw(s->boids, *(s->params), 1);

    // for(const BoidBase & b : *(s->boids)){
    //   b.drawSelf(*(s->params));
    // }

    glRotatef(0.13, 1, 0, 0);
    glRotatef(0.17, 0, 1, 0);
    glRotatef(0.09 , 0, 0, 1);

    s->params->DrawSimBBox();

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

    std::cout <<
                "Draws max/real fps:\t" << 
                                              1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(draw_delta_ticks).count()  <<
                "   /  " <<                              
                                              1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(frame_delta_ticks).count() <<
                '\n' <<
    std::endl;

  }

}


int main(int argc, char *argv[]){



  GL_window = CreateOpenWindow(
                                1920,//1080,
                                1080
                            );

  std::thread worker(nextGen);

  showBoids();
 
  std::cout << "!!!END!!!" << std::endl;

}