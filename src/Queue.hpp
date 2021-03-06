#pragma once
#include <mutex>              
#include <condition_variable>
#include <memory>
#include "Boid.hpp"

template <typename T>
class Queue{

    protected:

        using TPtr = std::shared_ptr<T>;

        TPtr elem;
        std::mutex mtx;
        std::condition_variable cv_push;
        std::condition_variable cv_pop;



    public:

        void pushWait(const TPtr & e){

            {
                std::unique_lock<std::mutex> lck(this->mtx);
                this->cv_pop.wait( lck, [&](){ return !this->elem; } );
                this->elem = e;
            }
            this->cv_push.notify_one();
        }

        TPtr pushOverwite(const TPtr & e){

            TPtr r;

            {
                std::scoped_lock<std::mutex> lck(this->mtx);
                r = std::exchange(this->elem, e);
            }

            this->cv_push.notify_one();

            return r;
        }


        TPtr popWait(){
            TPtr r;

            {
                std::unique_lock<std::mutex> lck(this->mtx);
                this->cv_push.wait( lck, [&](){ return this->elem; } );
                r = std::move(this->elem);
            }

            this->cv_pop.notify_one();
            return r;
        }

        TPtr popTry(){
            TPtr r;
            {
                std::scoped_lock<std::mutex> lck(this->mtx);
                r = std::move(this->elem);
            }

            this->cv_pop.notify_one();
            return r;
        }

        TPtr popGet(){
            std::scoped_lock<std::mutex> lck(this->mtx);
            return this->elem;
        }

};