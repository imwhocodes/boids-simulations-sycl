#pragma once
#include <memory>
#include <vector>
#include <CL/sycl.hpp>
#include "GLWindow.hpp"



float_t randFloat() {
    return float_t(rand()) / float_t(RAND_MAX);
}

float_t randFloat(const float_t max) {
    return randFloat() * max;
}

float_t randFloat(const float_t min, const float_t max) {
    return min + randFloat(max - min);
}

using GCoord_t = sycl::float3;


class SimulationParams {

    public:
      using Ptr = std::shared_ptr<SimulationParams>;
      using Coord_t = GCoord_t;

    protected:
      Coord_t _half_bbox_size;
      Coord_t _weights;
      float_t _max_speed;
      float_t _acceleration;
      float_t _bbox_repel_radius;

    public:
      SimulationParams(const Coord_t & half_bbox_size, const Coord_t & weights, const float_t max_speed, const float_t acceleration, const float_t bbox_repel_radius)
        :
      _half_bbox_size{half_bbox_size},
      _weights{weights},
      _max_speed{max_speed},
      _acceleration{acceleration},
      _bbox_repel_radius{bbox_repel_radius}
      {}


      void SetSimulationSize(const Coord_t & size){   
        this->_half_bbox_size = size / 2;
      }


      const auto & half_bbox_size() const {
        return this->_half_bbox_size;
      }


      const auto & acceleration() const {
        return this->_acceleration;
      }

      const auto & max_speed() const {
        return this->_max_speed;
      }

      const auto & bbox_repel_radius() const {
        return this->_bbox_repel_radius;
      }

      const auto & separation_weight() const {
        return this->_weights[0];
      }

      const auto & perception_weight() const {
        return this->_weights[1];
      }

      const auto & alignment_weight() const {
        return this->_weights[2];
      }

      void DrawSimBBox() const {
        constexpr float_t z_near = 1;
        constexpr float_t z_far = 100;
        constexpr float_t fov = 70;

        constexpr float_t aspect = 1;

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        const Coord_t min = -this->_half_bbox_size;
        const Coord_t max =  this->_half_bbox_size;

        // glFrustum(min.x(), max.x(), min.y(), max.y(), max.z(), max.z() * 3);

        // glPerspective(90, 1, 1, 100);

        // glFrustum(1.0, -1.0, -1.0, 1.0, 0.1, 1);

        const float_t fH = std::tan( fov / 360.0f * M_PI ) * z_near;
        const float_t fW = fH * aspect;

        glFrustum( -fW, fW, -fH, fH, z_near, z_far );

        glTranslatef(0, 0, min.z() * 3);
        
        glMatrixMode(GL_MODELVIEW);
        DrawCube(-this->_half_bbox_size, this->_half_bbox_size);
      }
};




class BoidBase {

  private:
    class NextGenKernel;


  public:
    using Coord_t = GCoord_t;

    using Vector_t = std::vector<BoidBase>;
    using VectorPrt_t = std::shared_ptr<Vector_t>;

    Coord_t position;
    Coord_t velocity;
    Coord_t acceleration;

    BoidBase() = default;

    BoidBase(const Coord_t & position, const Coord_t & velocity)
      :
    position{position},
    velocity{velocity}
    {}


    BoidBase(const Coord_t & position, const Coord_t & velocity, const Coord_t & acceleration)
      :
    position{position},
    velocity{velocity},
    acceleration{acceleration}
    {}

    static BoidBase Random(const SimulationParams & param){

      return { 
                                  Coord_t{
                                            randFloat(-param.half_bbox_size().x(), param.half_bbox_size().x()),
                                            randFloat(-param.half_bbox_size().y(), param.half_bbox_size().y()),
                                            randFloat(-param.half_bbox_size().z(), param.half_bbox_size().z())
                                        },
                // param.half_bbox_size() * randFloat(-1, 1),
                BoidBase::ClampVelocity(
                                          {
                                            randFloat(-param.max_speed(), param.max_speed()),
                                            randFloat(-param.max_speed(), param.max_speed()),
                                            randFloat(-param.max_speed(), param.max_speed())
                                          },
                                          param
                                        )
            };
    }



    Coord_t getForceFromBBox(const SimulationParams & param) const {


      const Coord_t pos_sign = sycl::sign( this->position );

      const Coord_t pos_abs = this->position * pos_sign;

      const Coord_t inv_abs_clamped_pos = sycl::min(param.half_bbox_size() - pos_abs, param.bbox_repel_radius());

      const Coord_t inv_abs_alpha = Coord_t{Coord_t::element_type{1}} - (inv_abs_clamped_pos / param.bbox_repel_radius());
      
      return - (inv_abs_alpha * pos_sign);
    }

    void drawSelf(const SimulationParams & param) const {

      const float_t wt = 1 ; //Boid::PERCEPTION_WEIGHT + Boid::SEPARATION_WEIGHT + Boid::SEPARATION_WEIGHT;

      const float_t f = std::min(sycl::length(this->acceleration) / ( param.acceleration() * wt), 1.0f);

      const Color_t hsv{std::lerp(0.5f, 0.0f, f), 1, 1};

      const Color_t rgb = hsv2rgb(hsv);

      // std::cout << "\n\n" << hsv.transpose() << '\n' << rgb.transpose() << std::endl;

      glColor3f(rgb.x(), rgb.y(), rgb.z());
      DrawFilledCircle(
                          this->position.x(),
                          this->position.y(),
                          this->position.z(),
                          0.005
      );
    }

    inline BoidBase applyForce(const Coord_t & force, const SimulationParams & param) const {

      const Coord_t acc = force * param.acceleration();

      return { 
                BoidBase::ClampPosition( this->position + this->velocity, param ),
                BoidBase::ClampVelocity( this->velocity + acc, param),
                acc
            };
    }


    inline static Coord_t IfNaNGetZero(const Coord_t & in){
      for(size_t i = 0; i < Coord_t::get_count(); i++){
        if(std::isnan(in[i])){
          return Coord_t{Coord_t::element_type{0}};
        }
      }

      return in;
    }

    inline static Coord_t ClampVelocity(const Coord_t & velocity, const SimulationParams & param){
            
      const float_t abs_vel = sycl::length(velocity);

      if(abs_vel <= param.max_speed()){
          return velocity;
      }

      else{
          return velocity * (param.max_speed() / abs_vel);
      }

    }

    inline static Coord_t ClampPosition(const Coord_t & position, const SimulationParams & param){
      return sycl::clamp(position, -param.half_bbox_size(), param.half_bbox_size());
    }


    // static Coord_t WarpPosition(const Coord_t & position, const SimulationParams & param){

    //   return sycl::fmod(position + param.half_bbox_size() * 2, param.half_bbox_size());
    // }



    // Coord_t getSeparationForce(const Coord_t & center, const float_t radius) const {
        
    //     Coord_t res = Coord_t{ Coord_t::element_type{0} };

    //     const Coord_t dist_vector = center - this->position;

    //     for(size_t i = 0; i < Coord_t::get_count(); i++){

    //       const float_t dist = dist_vector[i];

    //       if(!std::isnan(dist)){
    //         if(dist > 0){
    //           res[i] = - ( 1 - (dist / radius) );
    //         }
    //         else if (dist < 0){
    //           res[i] = 1 + (dist / radius);
    //         }  
    //       }
    //     }

      
    //   // assert(sycl::length(res) <= 1);

    //   return sycl::normalize(res);
    // }


    inline Coord_t getSeparationForce(const Coord_t & center, const float_t & radius) const {

      const Coord_t dist_vector = center - this->position;

      const Coord_t dist_vector_alpha = dist_vector / radius;

      const float_t alpha_dist = sycl::length(dist_vector_alpha);

      assert(alpha_dist <= 1.1 || std::isnan(alpha_dist));

      const float_t inv_force = alpha_dist - 1 ;

      const Coord_t force = (dist_vector_alpha * inv_force);

      assert(sycl::length(force) <= 1 || std::isnan(alpha_dist));

      return IfNaNGetZero( force );
    }


    inline Coord_t getPerceptionForce(const Coord_t & center, const float_t & radius) const {
      return IfNaNGetZero( ( center - this->position) / radius );
    }

    inline Coord_t getAlignmentForce(const Coord_t & center, const float_t & max_speed) const {
      return IfNaNGetZero( ( center - this->velocity ) / (max_speed * 2) );
    }


    //MAYBE SPAW POINTER
    inline static VectorPrt_t ComputeNextGen( const VectorPrt_t & boids_in, const SimulationParams::Ptr & param, sycl::queue & queue){

      const size_t boid_nums_cpu = boids_in->size();

      VectorPrt_t boids_out = std::make_shared<Vector_t>( boid_nums_cpu );
      
      { // start of scope, ensures data copied back to host

        sycl::buffer<BoidBase, 1> boids_in_buffer  ( boids_in->data(),   sycl::range<1>(boid_nums_cpu) );
        sycl::buffer<BoidBase, 1> boids_out_buffer ( boids_out->data(),  sycl::range<1>(boid_nums_cpu) );
        // sycl::buffer<SimulationParams, 1> sim_params_buffer  ( sim_params, sycl::range<1>(1));

        queue.submit([&] (sycl::handler& cgh) {

          const size_t boid_nums_gpu = boid_nums_cpu;

          const auto boids_in_acc   = boids_in_buffer   .get_access<sycl::access::mode::read>(cgh);
          const auto boids_out_acc  = boids_out_buffer  .get_access<sycl::access::mode::discard_write>(cgh);
          // const auto sim_params_acc = sim_params_buffer .get_access<sycl::access::mode::read>(cgh);
        
          const SimulationParams sim_params_gpu = *param;

          // const Coord_t norm_vector = sycl::normalize(Coord_t{ Coord_t::element_type{1} });

          cgh.parallel_for<NextGenKernel>( sycl::range<1>(boid_nums_gpu), [=](sycl::id<1> ic)
            {
              const BoidBase cboid = boids_in_acc[ic];
            
              const Coord_t cvel_norm = sycl::normalize(cboid.velocity);


              const float_t separation_max_radius = 0.10;
              Coord_t separation_tot = - cboid.position;// Coord_t{ Coord_t::element_type{0} }; //;
              size_t  separation_cnt = 0;


              const float_t perception_max_radius = 0.35;
              const float_t perception_max_angle = 0.3;// 0.8;
              Coord_t perception_tot = Coord_t{ Coord_t::element_type{0} } ;//- cboid.position;
              size_t  perception_cnt = 0;


              const float_t alignment_max_radius = 0.55;
              const float_t alignment_max_angle = 0.5;
              Coord_t alignment_tot  = Coord_t{ Coord_t::element_type{0} }; //- cboid.velocity;
              size_t  alignment_cnt  = 0;


              for(size_t il = 0; il < boid_nums_gpu; il++){

                const BoidBase lboid = boids_in_acc[il];

                const Coord_t distance_vect = cboid.position - lboid.position;

                const float_t distance_norm = cl::sycl::length(distance_vect);

                const float_t distance_angle = sycl::dot(distance_vect, cvel_norm) / distance_norm; //NAN for SELF

                const float_t distance_angle_abs = distance_angle * sycl::sign(distance_angle);


                // const bool b_sepation = distance_norm < separation_max_radius;
                // separation_tot += bpos * b_sepation;
                // separation_cnt += b_sepation;
                if(distance_norm <= separation_max_radius){
                  separation_tot += lboid.position;
                  separation_cnt++;
                }


                // const bool b_perception = distance_norm < perception_max_radius && distance_angle < perception_max_angle;
                // perception_tot += bpos * b_perception;
                // perception_cnt += b_perception;
                if(distance_norm <= perception_max_radius && distance_angle_abs <= perception_max_angle){
                  perception_tot += lboid.position;
                  perception_cnt++;
                }


                // const bool b_alignment = distance_norm < alignment_max_radius && distance_angle < alignment_max_angle;
                // alignment_tot += bvel * b_alignment;
                // alignment_cnt += b_alignment;
                if(distance_norm <= alignment_max_radius && distance_angle_abs <= alignment_max_angle){
                  alignment_tot += lboid.velocity;
                  alignment_cnt++;
                }

              }


              assert((separation_cnt - 1) < boid_nums_gpu);
              // assert((perception_cnt - 1) < boid_nums_gpu);
              // assert((alignment_cnt  - 1) < boid_nums_gpu);

              const Coord_t separation_center = separation_tot / (separation_cnt - 1);
              const Coord_t perception_center = perception_tot / (perception_cnt - 0);
              const Coord_t alignment_center  = alignment_tot  / (alignment_cnt  - 0);



              // const Coord_t separation_vect = separation_center - cboid.position;

              // const Coord_t separation_vect_alpha = eparation_vect / separation_max_radius ;

              // const Coord_t separation_vect_sign = sycl::sign(separation_vect);

              // separation_vect_off = separation_vect + 
              // // const Coord_t separation_vect_abs = separation_vect * separation_vect_sign;

              // // const Coord_t separation_force =   - ( ( separation_vect_abs / separation_max_radius ) - norm_vector ) * separation_vect_sign;

              // // const Coord_t separation_force = sycl::max(separation_vect, Coord_t{ Coord_t::element_type{ 0 } } ) - norm_vector;

              const Coord_t bbox_force = cboid.getForceFromBBox(sim_params_gpu);


              const Coord_t separation_force = cboid.getSeparationForce( separation_center, separation_max_radius );

              const Coord_t perception_force = cboid.getPerceptionForce( perception_center, perception_max_radius );

              const Coord_t alignment_force =  cboid.getAlignmentForce ( alignment_center,  sim_params_gpu.max_speed() );


              assert(sycl::length(separation_force) <= 1.1);
              assert(sycl::length(perception_force) <= 1.1);
              assert(sycl::length(alignment_force)  <= 1.1);
              assert(sycl::length(bbox_force)       <= 2.1);


              const Coord_t force = 
                                    bbox_force
                                      +
                                    separation_force
                                      +
                                    perception_force
                                      +
                                    alignment_force
                                  ;

              assert(!std::isnan(force.x()));
              assert(!std::isnan(force.y())); 
              assert(!std::isnan(force.z()));

              boids_out_acc[ic] = cboid.applyForce(force, sim_params_gpu);

            }
          );

        });

      } // end of scope, ensures data copied back to host

      return boids_out;
    }

};


