#pragma once


#include <stdint.h>
#include <vector>
#include <functional>
#include <iostream>
#include <optional>

#include <rtmidi/RtMidi.h>

class MidiCommand {

    public:
        enum class Type : uint8_t {
            Unknow,

            KeyPress,
            KeyRelease,

            PadPress,
            PadRelease,

            KnobValue
        };

        using List_t = std::vector<MidiCommand>;

    protected:
        const MidiCommand::Type type;
        const uint8_t input;
        const uint8_t value;
        
    public:



        MidiCommand(const MidiCommand::Type type, const uint8_t input, const uint8_t value)
            :
        type{type},
        input{input},
        value{value}
        {}

        bool isKey() const {
            return 
                    this->type == MidiCommand::Type::KeyPress
                        ||
                    this->type == MidiCommand::Type::KeyRelease
                ;
        }

        bool isPad() const {
            return 
                    this->type == MidiCommand::Type::PadPress
                        ||
                    this->type == MidiCommand::Type::PadRelease
                ;
        }

        bool isKnob() const {
            return this->type == MidiCommand::Type::KnobValue;
        }


        const MidiCommand::Type & getType() const {
            return this->type;
        }

        const uint8_t & getInput() const {
            return this->input;
        }

        const uint8_t & getValue() const {
            return this->value;
        }


        float_t getValueNorm() const {
            return this->value / float_t(127);
        }

        template<typename T>
        float_t getValueSigma(const T sigma) const {

            return 1 - std::pow( 1 - this->getValueNorm(), sigma);

        }




        static MidiCommand::Type DecodeByteToType(const uint8_t input){
            switch (input){
                case 144:
                    return MidiCommand::Type::KeyPress;
                case 128:
                    return MidiCommand::Type::KeyRelease;


                case 145:
                    return MidiCommand::Type::PadPress;
                case 129:
                    return MidiCommand::Type::PadRelease;     


                case 176:
                    return MidiCommand::Type::KnobValue;     


                default:
                    return MidiCommand::Type::Unknow;
            }
        }

        static MidiCommand::List_t KnobMergedLastValue(const MidiCommand::List_t & input) {

            MidiCommand::List_t res;

            for(auto it = input.rbegin(); it != input.rend(); ++it){

                const MidiCommand & current = *it;

                if(current.isKnob()){

                    bool will_push_current = true;

                    for(const MidiCommand & stored : res){

                        if(stored.getInput() == current.getInput()){
                            will_push_current = false;
                            break;
                        }

                    }

                    if(will_push_current){
                        res.push_back(current);
                    }

                }

            }

            return res;

        }
};


class MidiDecoder {
    protected:

        RtMidiIn midi_in;

        using RawBuffer_t = std::vector<unsigned char>;

        MidiDecoder::RawBuffer_t getRawBuffer() {
            std::vector<unsigned char> message;
            this->midi_in.getMessage( &message );
            return message;
        }


    public:
        MidiDecoder()
            :
        midi_in{RtMidi::Api::UNSPECIFIED, "RtMidi Input Client", 1024}
        {}


        void open(const size_t port, const size_t buffer_size = 1024, const size_t buffer_count = 4){
            this->midi_in.setBufferSize(buffer_size, buffer_count);
            this->midi_in.openPort(port);
            // Don't ignore sysex, timing, or active sensing messages.
            this->midi_in.ignoreTypes( false, false, false );
        }


        std::optional<MidiCommand> getRawCommand() {
            const MidiDecoder::RawBuffer_t raw_buffer = this->getRawBuffer();

            switch (raw_buffer.size()){
                case 0:
                    return std::nullopt;

                case 3:
                    return MidiCommand{ MidiCommand::DecodeByteToType(raw_buffer[0]), raw_buffer[1], raw_buffer[2] };
            
                default:
                    std::cerr << "MidiDecoder: unrecognised command:\t" << int(raw_buffer[0]) << std::endl;
                    return MidiCommand{ MidiCommand::Type::Unknow, 0, 0 };
            }

        }


        MidiCommand::List_t getRawCommands() {

            MidiCommand::List_t res;

            while (true){

                std::optional<MidiCommand> cmd = this->getRawCommand();

                if(cmd){
                    res.push_back(*cmd);
                }

                else{
                    return res;
                }
               
            }

        }

        // void addCallback( const std::function<void(const MidiDecoder::List_t &)> ){



        // }


};


// try {

// }

// catch (RtMidiError &error) {
//     error.printMessage();
// }