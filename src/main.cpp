#include "Boid.hpp"
#include "GLWindow.hpp"
#include <iostream>
#include <ctime>

#include <chrono>
#include <thread>

#include "RtMidi.h"

#include "MidiDecoder.hpp"

int main(int argc, char *argv[]){

    srand(0);

   

    // Check inputs.

    MidiDecoder akai_mpk;


    try {
        akai_mpk.open(0);
    }

    catch (const RtMidiError & error) {
        error.printMessage();
    }


    GLFWwindow * window = CreateOpenWindow(
                                                1920,
                                                1080
                                            );

    /* Make the window's context current */
    

    Boid::SetSimulationSize(Boid::Coord_t{16.0 / 9.0, 1} * 3);


    Boid::PtrList_t boids_list = Boid::RandomPtrList(150);




    // Boid::Coord_t test = Boid::Coord_t::Random()



    constexpr float_t FPS_TARGET = 30;

    constexpr std::chrono::nanoseconds FPS_PERIOD_TARGET = std::chrono::microseconds(int(1000 / FPS_TARGET)) ;

    using Clock_t = std::chrono::high_resolution_clock;

    auto last_frame_ticks_real = Clock_t::now();
    auto next_frame_ticks_virtual = last_frame_ticks_real;


    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window)){

        std::cout << "\n\n\n//////////////\n";

        const MidiCommand::List_t midi_cmds = akai_mpk.getRawCommands();

        const MidiCommand::List_t midi_knobs =  MidiCommand::KnobMergedLastValue(midi_cmds);


        std::cout << "KnobMerged:\t" << midi_knobs.size() << '\n';

        for(const MidiCommand & c : midi_knobs){
            std::cout << "Knob:\t" << int(c.getInput()) << "\tValue:\t" << int(c.getValue()) << "\t( " << c.getValueNorm() << " )\n";

            switch (c.getInput()){

                case 1: {
                            const float_t new_qty = std::lerp(
                                                                float_t(Boid::ABS_MIN_BOID_QTY),
                                                                float_t(Boid::ABS_MAX_BOID_QTY),
                                                                c.getValueSigma(0.5)
                                                            );
                            
                            boids_list = Boid::UpdateBoidQuantity(boids_list, std::round(new_qty));
                            break;
                        }

                case 5: {
                    boids_list = Boid::SetSimulationSize(Boid::Coord_t{16.0 / 9.0, 1} * std::lerp(1.0f, 10.0f, c.getValueNorm()), boids_list);
                    break;
                }
                
                default:
                    break;
            }
        }

        const auto computing_start_ticks = Clock_t::now();

        boids_list = Boid::MainComputeNewGeneration_V2(boids_list);

        const auto computing_end_ticks = Clock_t::now();



        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers

        next_frame_ticks_virtual = next_frame_ticks_virtual + FPS_PERIOD_TARGET;


        const auto check_sleep = Clock_t::now();

        if(check_sleep < next_frame_ticks_virtual) {

            std::this_thread::sleep_for( next_frame_ticks_virtual - check_sleep);

            while(Clock_t::now() < next_frame_ticks_virtual);
        }



        const auto draw_start_ticks = Clock_t::now();

        for(const auto & b : boids_list){
            b->drawSelf();
        }

        const auto draw_end_ticks = Clock_t::now();


        const auto draw_delta_ticks = draw_end_ticks - draw_start_ticks;


        // int width, height;
        // glfwGetFramebufferSize(window, &width, &height);

        // glColor3f(1.0, 0, 1.0);

        // glBegin(GL_TRIANGLES);
        //     glVertex3f(0.0, 0.0, 0);
        //     glVertex3f(width, 0.0, 0);
        //     glVertex3f(0.0, height, 0);
        // glEnd();

        /* Swap front and back buffers */
        glfwSwapBuffers(window);


        /* Poll for and process events */
        glfwPollEvents();

        const auto e = glGetError();

        if(e){
            std::cout << glErrorToString(e) << std::endl;
        }


        const auto computing_delta_ticks = computing_end_ticks - computing_start_ticks;


        const auto now = Clock_t::now();

        const auto frame_delta_ticks = now - last_frame_ticks_real;

        last_frame_ticks_real = now;

        std::cout <<
            "Compu fps:\t" << 1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(computing_delta_ticks).count() << '\n' <<
            "Draws fps:\t" << 1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(draw_delta_ticks).count()      << '\n' <<
            "Real  fps:\t" << 1e+9 / std::chrono::duration_cast<std::chrono::nanoseconds>(frame_delta_ticks).count()     << '\n' <<

        std::endl;

    }

    glfwTerminate();

    return 0;
}