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



constexpr size_t BOIDS_NUM = 1'000;//45'000;//20'000;//3000000;

constexpr std::chrono::nanoseconds FPS_PERIOD_TARGET = std::chrono::nanoseconds(int(1.0E9 / 60.0)) ;


Queue<SimSample> sample_queue;
GLFWwindow * GL_window;


void nextGen(){

  // sycl::gpu_selector device_selector;
  sycl::default_selector device_selector;
  sycl::queue gpu_queue(device_selector);
  std::cout << "Running on " << gpu_queue.get_device().get_info<sycl::info::device::name>() << "\n";


  SimulationParams::Ptr sim_params_ptr = std::make_shared<SimulationParams>(
    SimulationParams::Coord_t{1, 1, 1},
    SimulationParams::Coord_t{1, 1, 1},
    0.025,
    0.005,
    0.15
  );


  sim_params_ptr->SetSimulationSize(SimulationParams::Coord_t{SimulationParams::Coord_t::element_type{5}});


  BoidBase::VectorPrt_t last_simulation_boids  = std::make_shared<BoidBase::Vector_t>( BOIDS_NUM );

  for(BoidBase & b : *last_simulation_boids){
    b = BoidBase::Random(*sim_params_ptr);
  } 


  auto last_frame_ticks_real = Clock_t::now();

  while (!glfwWindowShouldClose(GL_window)){

    const auto start_computing = Clock_t::now();

    BoidBase::VectorPrt_t boids_out = BoidBase::ComputeNextGen(last_simulation_boids, sim_params_ptr, gpu_queue);

    const auto end_computing = Clock_t::now();

    const auto computing_delta_ticks = end_computing - start_computing;


    sample_queue.pushWait(std::make_shared<SimSample>(boids_out, sim_params_ptr));

    last_simulation_boids = boids_out;

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

    for(const BoidBase & b : *(s->boids)){
      b.drawSelf(*(s->params));
    }

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
                                1080,
                                1080
                            );

  std::thread worker(nextGen);

  showBoids();
 
  std::cout << "!!!END!!!" << std::endl;

}