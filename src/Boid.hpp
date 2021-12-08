#pragma once
#include <Eigen/Dense>
#include <vector>
#include "GLWindow.hpp"

#include <memory>

float_t randFloat() {
    return float_t(rand()) / float_t(RAND_MAX);
}

float_t randFloat(const float_t max) {
    return randFloat() * max;
}

float_t randFloat(const float_t min, const float_t max) {
    return min + randFloat(max - min);
}



class Boid 
    :
public std::enable_shared_from_this<Boid>{

    public:

        using Coord_t = Eigen::Vector3f;
        using CoordList_t = Eigen::Matrix3Xf;
        using DistanceList_t = Eigen::Matrix3Xf;

        using Ptr_t = std::shared_ptr<Boid>;
        using List_t = std::vector<Boid>;
        using PtrList_t = std::vector<Ptr_t>;

        static Coord_t HALF_SIMULATION_SIZE;

        static constexpr float_t BOID_SIZE  =  0.005;

        static float_t PERCEPTION_RADIUS;
        static float_t SEPARATION_RADIUS;
        static float_t ALIGNMENT_RADIUS;
        static constexpr float_t BBOX_RADIUS =        0.1;


        static float_t PERCEPTION_WEIGHT;
        static float_t SEPARATION_WEIGHT;
        static float_t ALIGNMENT_WEIGHT;
        static constexpr float_t BBOX_WEIGHT =        1;


        static constexpr float_t MAX_SPEED = 0.025;
        static constexpr float_t ACCELERATION = 0.005;


        static constexpr size_t ABS_MIN_BOID_QTY = 5;
        static constexpr size_t ABS_MAX_BOID_QTY = 4000;



    protected:

        const Coord_t position;
        const Coord_t velocity;
        const Coord_t acceleration;

        const float_t perception_radius;
        const float_t separarion_radius;
        const float_t  alignment_radius;



        static void assertPosition(const Coord_t & pos){
            assert(
                (
                    ( pos.array() >= -Boid::HALF_SIMULATION_SIZE.array() )
                        &&
                    ( pos.array() <=  Boid::HALF_SIMULATION_SIZE.array() )
                ).all()
            );
        }

        static void assertVelocity(const Coord_t & vel){
            assert(vel.norm() <= (Boid::MAX_SPEED + 0.01));
        }
    
    public:

        // Boid(const float_t x, const float_t y)
        //     :
        // Boid{Coord_t{x, y}, Coord_t::Zero()}
        // {}

        Boid(const Coord_t & pos, const Coord_t & vel, const float_t perception_radius, const float_t separarion_radius, const float_t alignment_radius)
            :
        position{pos},
        velocity{vel},
        acceleration{Boid::Coord_t::Zero()},
        perception_radius{perception_radius},
        separarion_radius{separarion_radius},
        alignment_radius{alignment_radius}
        {
            Boid::assertPosition(pos);
            Boid::assertVelocity(vel);
        }

        Boid(const Boid & boid, const Coord_t & acceleration )
            :
        position{ Boid::ClampPosition( boid.position + boid.velocity ) },
        velocity{ Boid::ClampVelocity( boid.velocity + acceleration ) },
        acceleration{acceleration},
        perception_radius{boid.perception_radius},
        separarion_radius{boid.separarion_radius},
        alignment_radius{boid.alignment_radius}
        {

        }

        const Coord_t & getPosition() const {
            return this->position;
        }


        Boid applyForce(const Coord_t & force) const {

            return this->applyAcceleration(force * Boid::ACCELERATION);
        }

        Boid applyAcceleration(const Coord_t & acceleration) const {

            return Boid{ *this, acceleration };
        }

        Boid applyNoForce() const {
            return this->applyAcceleration(Coord_t::Zero());
        }


        // float_t distance(const Boid & other) const {
        //     return (this->position - other.position).norm();
        // }

        void drawSelf() const {

            const float_t wt = 1 ; //Boid::PERCEPTION_WEIGHT + Boid::SEPARATION_WEIGHT + Boid::SEPARATION_WEIGHT;

            const float_t f = std::min(this->acceleration.norm() / ( Boid::ACCELERATION * wt), 1.0f);

            const Color_t hsv{std::lerp(0.5f, 0.0f, f), 1, 1};

            const Color_t rgb = hsv2rgb(hsv);

            // std::cout << "\n\n" << hsv.transpose() << '\n' << rgb.transpose() << std::endl;

            glColor3f(rgb.x(), rgb.y(), rgb.z());
            DrawFilledCircle(
                                this->position.x(),
                                this->position.y(),
                                this->position.z(),
                                Boid::BOID_SIZE / 2
            );
        }

        static void DrawSimBBox() {
            DrawCube(-Boid::HALF_SIMULATION_SIZE, Boid::HALF_SIMULATION_SIZE);
        }


        Coord_t getForceFromBBox() const { 

            // const float_t radius = Eigen::Vector3f{
            //                                             Boid::PERCEPTION_RADIUS,
            //                                             Boid::SEPARATION_RADIUS,
            //                                             Boid::ALIGNMENT_RADIUS
            //                                     }.maxCoeff();

            return - ( Coord_t::Ones() - ( ( Boid::HALF_SIMULATION_SIZE - this->position.cwiseAbs() ).cwiseMin( Boid::BBOX_RADIUS ) / Boid::BBOX_RADIUS ) ).cwiseProduct(this->position.cwiseSign());

        }


        static Boid Random(){
            return { 
                        Boid::Coord_t::Random().cwiseProduct((Boid::HALF_SIMULATION_SIZE - Boid::Coord_t::Constant(Boid::BBOX_RADIUS))),
                        Boid::ClampVelocity( Coord_t::Random() * Boid::MAX_SPEED ),
                        randFloat(Boid::BOID_SIZE * 5, 1),
                        randFloat(Boid::BOID_SIZE * 2, 0.3),
                        randFloat(Boid::BOID_SIZE * 7, 1.1)
            };
        }

        static Coord_t ClampPosition(const Coord_t & position){
            return position.cwiseMax(-Boid::HALF_SIMULATION_SIZE).cwiseMin(Boid::HALF_SIMULATION_SIZE);
        }

        static Coord_t ClampVelocity(const Coord_t & velocity){

            const float_t abs_vel = velocity.norm();

            if(abs_vel <= Boid::MAX_SPEED){
                return velocity;
            }

            else{
                return velocity * (Boid::MAX_SPEED / abs_vel);
            }


            
        }


        static Boid::PtrList_t RandomPtrList(const size_t n){

            Boid::PtrList_t res;

            for (size_t i = 0; i < n; i++){

                res.push_back(std::make_shared<Boid>(Boid::Random()));
            }
            
            return res;

        }

        static Boid::PtrList_t UpdateBoidQuantity(const Boid::PtrList_t & input, const size_t qty){

            const size_t in_size = input.size();

            if(in_size < qty){

                Boid::PtrList_t res = input;

                const size_t to_add = qty - in_size;

                for(size_t i = 0; i < to_add; i++ ){
                    res.push_back(std::make_shared<Boid>(Boid::Random()));
                }

                return res;
            }

            else if(in_size > qty){

                Boid::PtrList_t res;
                res.reserve(qty);

                const size_t to_remove = in_size - qty;


                const size_t skip_every = ( (in_size + (to_remove-1) ) / to_remove );

                for(size_t i = 0; (i < in_size) && (res.size() < qty); i++ ){ // 

                    if( i % skip_every ){
                        res.push_back(input[i]);
                    }
                }

                assert(res.size() == qty);

                return res;

            }

            else{
                return input;
            }

            
        }

        static Boid::CoordList_t GeneratePositionList(const Boid::PtrList_t & input){
            Boid::CoordList_t res;

            res.resize(Eigen::NoChange, input.size());

            for(size_t i = 0; i < input.size(); i++){
                res.col(i) = input[i]->position;
            }

            return res;

        }


        static Boid::CoordList_t GenerateVelocityList(const Boid::PtrList_t & input){
            Boid::CoordList_t res;

            res.resize(Eigen::NoChange, input.size());

            for(size_t i = 0; i < input.size(); i++){
                res.col(i) = input[i]->velocity;
            }

            return res;

        }

        static Boid::Coord_t IfNaNGetZero( const Boid::Coord_t & v ){
            if(v.array().isNaN().any()){
                return Boid::Coord_t::Zero();
            }
            else{
                return v;
            }
        }


        static Boid::DistanceList_t GenerateDistanceList(const Boid::CoordList_t & positions, const Boid::CoordList_t & directions, const Eigen::Index idx ){

            Boid::DistanceList_t res;

            assert(positions.cols() == directions.cols());

            res.resize(Eigen::NoChange, positions.cols());

            const auto pi = positions.col(idx);
            const auto vi = directions.col(idx);


            const auto dist_vect = positions.colwise() - pi;
            const auto dist_norm = dist_vect.colwise().norm();


            const auto angles = ( dist_vect.array() * vi.normalized().replicate(1, directions.cols()).array() ).colwise().sum().abs() / dist_norm.array();


            assert(dist_norm.rows() == 1);


            res.row(0) = dist_norm;
            res.row(1) = angles;

            // std::cout << res.row(1) << std::endl;

            res.col(idx).array() = INFINITY;

            
            return res;

        }


        static Boid::Coord_t ComputeAvgInRadiusAngle(const Boid::DistanceList_t & distances, const Boid::CoordList_t & values, const float_t max_distance, const float_t max_angle){
            
            const auto mask = (distances.row(0).array() <= max_distance) && (distances.row(1).array() <= max_angle);

            const auto mask_sized = mask.replicate< Boid::Coord_t::RowsAtCompileTime, 1 >();

            const auto masked = mask_sized.select(values, 0);

            return  masked.rowwise().sum() / mask.count();

        }


        static Boid::PtrList_t MainComputeNewGeneration(const Boid::PtrList_t & input){

            Boid::PtrList_t res(input.size());

            const Boid::CoordList_t pos_list = Boid::GeneratePositionList(input);
            const Boid::CoordList_t vel_list = Boid::GenerateVelocityList(input);

            assert(!pos_list.array().isNaN().any());
            assert(!vel_list.array().isNaN().any());


            #pragma omp parallel for schedule(static)
            for(size_t i = 0; i < input.size(); i++ ){

                const Boid & b = *(input[i]);

                const Boid::DistanceList_t dist_list = Boid::GenerateDistanceList(pos_list, vel_list, i);


                const Boid::Coord_t force_of_bbox             = b.getForceFromBBox();



                const Boid::Coord_t center_of_group_perception =   Boid::ComputeAvgInRadiusAngle(dist_list, pos_list, b.perception_radius, 0.50);
                const Boid::Coord_t force_of_group_perception =     (center_of_group_perception - b.position) / b.perception_radius;

                
                
                const Boid::Coord_t center_of_group_separation = Boid::ComputeAvgInRadiusAngle(dist_list, pos_list, b.separarion_radius, INFINITY);
                const Boid::Coord_t aplha_to_group_separation =  (center_of_group_separation - b.position) / b.separarion_radius;
                const Boid::Coord_t force_of_group_separation = - (Coord_t::Ones().normalized() - aplha_to_group_separation.cwiseAbs()).array() * aplha_to_group_separation.cwiseSign().array();


    
                const Boid::Coord_t velocity_of_group_alignment =   Boid::ComputeAvgInRadiusAngle(dist_list, vel_list,b.alignment_radius, 0.75);
                const Boid::Coord_t force_of_group_velocity  =      (velocity_of_group_alignment - b.velocity) / (Boid::MAX_SPEED * 2);


                res[i] = std::make_shared<Boid>( 
                                                    b.applyForce( 
                                                                    (
                                                                        ( force_of_bbox * Boid::BBOX_WEIGHT )
                                                                            +
                                                                        ( Boid::IfNaNGetZero(force_of_group_perception) * Boid::PERCEPTION_WEIGHT )
                                                                            + 
                                                                        ( Boid::IfNaNGetZero(force_of_group_separation) * Boid::SEPARATION_WEIGHT )
                                                                            + 
                                                                        ( Boid::IfNaNGetZero(force_of_group_velocity)   * Boid::ALIGNMENT_WEIGHT )
                                                                    )
                                                                    //     /
                                                                    
                                                                    // (
                                                                    //     Boid::BBOX_WEIGHT
                                                                    //         +
                                                                    //     Boid::PERCEPTION_WEIGHT
                                                                    //         +
                                                                    //     Boid::SEPARATION_WEIGHT
                                                                    //         +
                                                                    //     Boid::ALIGNMENT_WEIGHT
                                                                    // )
                                                                ) 
                            );
               
            }

            return res;
        }


        // static void SetSimulationSize(const Coord_t & size){
        //     Boid::HALF_SIMULATION_SIZE = size / 2;

        //     glMatrixMode(GL_PROJECTION);
        //     glLoadIdentity();

        //     const Boid::Coord_t min = -Boid::HALF_SIMULATION_SIZE;
        //     const Boid::Coord_t max =  Boid::HALF_SIMULATION_SIZE;

        //     glOrtho(min.x(), max.x(), max.y(), min.y(), -1, 1);

        //     glMatrixMode(GL_MODELVIEW);
        // }

        static void SetSimulationSize(const Coord_t & size){
            
            constexpr float_t z_near = 1;
            constexpr float_t z_far = 100;
            constexpr float_t fov = 70;

            constexpr float_t aspect = 1;


            Boid::HALF_SIMULATION_SIZE = size / 2;

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();

            const Boid::Coord_t min = -Boid::HALF_SIMULATION_SIZE;
            const Boid::Coord_t max =  Boid::HALF_SIMULATION_SIZE;

            // glFrustum(min.x(), max.x(), min.y(), max.y(), max.z(), max.z() * 3);

            // glPerspective(90, 1, 1, 100);

            // glFrustum(1.0, -1.0, -1.0, 1.0, 0.1, 1);

            
            const float_t fH = std::tan( fov / 360.0f * M_PI ) * z_near;
            const float_t fW = fH * aspect;

            glFrustum( -fW, fW, -fH, fH, z_near, z_far );

            glTranslatef(0, 0, min.z() * 3);
           

            glMatrixMode(GL_MODELVIEW);
        }




        static Boid::PtrList_t SetSimulationSize(const Coord_t & size, const Boid::PtrList_t & input){
            
            const Coord_t old_half_size = Boid::HALF_SIMULATION_SIZE;
            
            Boid::SetSimulationSize(size);

            if( ( old_half_size.array() > Boid::HALF_SIMULATION_SIZE.array() ).any() ){

                Boid::PtrList_t res;

                for(const Boid::Ptr_t & b : input){
                    res.push_back( std::make_shared<Boid>( 
                                                            Boid::ClampPosition(b->position),
                                                            b->velocity,
                                                            b->perception_radius,
                                                            b->separarion_radius,
                                                            b->alignment_radius
                                                        )
                    );
                }

                return res;

            }

            else{
                return input;
            }  
            
        }


        static void SetPerceptionRadius(const float_t r){
            assert(r >= 0 && r <= 1);
            Boid::PERCEPTION_RADIUS =  r;
        }

        static void SetSeparationRadius(const float_t r){
            assert(r >= 0 && r <= 1);
            Boid::SEPARATION_RADIUS = r;
        }

        static void SetAlignmentRadius(const float_t r){
            assert(r >= 0 && r <= 1);
            Boid::ALIGNMENT_RADIUS =  r;
        }


        static void SetPerceptionWeight(const float_t w){
            assert(w >= 0 && w <= 1);
            Boid::PERCEPTION_WEIGHT =  w;
        }

        static void SetSeparationWeight(const float_t w){
            assert(w >= 0 && w <= 1);
            Boid::SEPARATION_WEIGHT = w;
        }

        static void SetAlignmentWeight(const float_t w){
            assert(w >= 0 && w <= 1);
            Boid::ALIGNMENT_WEIGHT =  w;
        }



};

// sizeof(Boid);
// static_assert(sizeof(Boid) == sizeof(float_t) * 6, "Check size");

Boid::Coord_t Boid::HALF_SIMULATION_SIZE = Boid::Coord_t::Constant(0.5);

float_t Boid::PERCEPTION_RADIUS =  0.1;
float_t Boid::SEPARATION_RADIUS =  Boid::PERCEPTION_RADIUS / 4;
float_t Boid::ALIGNMENT_RADIUS =   Boid::PERCEPTION_RADIUS * 3;

float_t Boid::PERCEPTION_WEIGHT =  0.6;
float_t Boid::SEPARATION_WEIGHT =  0.25;
float_t Boid::ALIGNMENT_WEIGHT =   0.7;