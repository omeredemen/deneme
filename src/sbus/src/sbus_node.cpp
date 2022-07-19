/*Bu node Joy topiğine subscriberdır. Herhangi bir mesaj 
yayınlamıyor.Sadece joystickten alınan mesajları SBUS 
mesajı halinde serial port üzerinden FC'a gönderiyor.
İleride sbus mesajlarını yayınlamak istediğimizde 
.msg dosyası oluşturmamız gerekecek.*/

#include "rclcpp/rclcpp.hpp" 
#include "sensor_msgs/msg/joy.hpp" 

#include "sbus.h" 

SBUS::SBusSerialPort sbus;

class sbus_node: public rclcpp::Node 
{
    public:
        sbus_node() : Node("sbus_node") 
        {
            //Subscriber
            sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
            std::bind(&sbus_node::getJoyMsg,this,std::placeholders::_1));

            //Belirlediğimiz frekansta SBUS mesajlarını seri port üzerinden gönderir.
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/freq),
            std::bind(&sbus_node::sendSbusMsg, this));
        }
        
    private:
        void getJoyMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
            //Joystickten eksen ve butonların değerlerini float 
            //tipinde joystick adlı bir diziye yerleştiririz.

            //Eksenler 
            //Roll ve Yaw eksenleri en sağda 1972 çıkışı 
            //vermesi için -1 ile çarpıldı.
            joystick[0]  = -joy_msg->axes[3];  //Roll
            joystick[1]  = joy_msg->axes[4];   //Pitch
            joystick[2]  = joy_msg->axes[1];   //Throttle 
            joystick[3]  = -joy_msg->axes[0];  //Yaw

            //R2 ve L2 hem eksen hem de buton olark kullanılabilir. 
            //Joy node'u bu eksenleri varsayılan olarak 1 olarak yayınlar .
            //"-1" ile çarpılır. Aksi takdirde R2 ve L2 eksenleri
            //normal halde dururken 1972 çıkışı alırız.
            joystick[4]  = -joy_msg->axes[2]; 
            joystick[5]  = -joy_msg->axes[5];  

            //Bu iki eksen yön tuşlarıdır. Ve sağ-sol yön tuşu "-1"
            //ile çarpılıp sağ yön tuşuna basıldığında 1972 olacak 
            //şekilde çıkış verilmesi sağlandı.
            joystick[6]  = -joy_msg->axes[6];  
            joystick[7]  = joy_msg->axes[7];   

            //Butonlar
            joystick[8]  = joy_msg->buttons[0]; // X butonu
            joystick[9]  = joy_msg->buttons[1]; // Daire butonu
            joystick[10] = joy_msg->buttons[2]; // Üçgen butonu
            joystick[11] = joy_msg->buttons[3]; // Kare butonu
            joystick[12] = joy_msg->buttons[4]; // L1 butonu
            joystick[13] = joy_msg->buttons[5]; // R1 butonu
            joystick[14] = joy_msg->buttons[6]; // L2 butonu default 0
            joystick[15] = joy_msg->buttons[7]; // R2 butonu default 0 
            
            //Linear Interpolation  y = y0 + ((y1-y0)/(x1-x0)) * (x - x0);
            //Eksenleri ve butonları 192 ile 1972 arasına yerleştirme denklemleri 
            for(int j = 0; j<16; j++){
                if(j<8){
                    joystick[j] = min_sbus + ((max_sbus-min_sbus)/(max_axes-min_axes)) * (joystick[j]-min_axes); 
                }
                else{
                    joystick[j] = min_sbus +((max_sbus-min_sbus)/(max_button-min_button)) * (joystick[j]-min_button); 
                }
                sbusChannels[j] = int(joystick[j]); //Joy mesajları float , SBUS kanalları integer
                                                    //veri tipinde olduğu için dönüştürme gereklidir. 
            }
        }

        //SBUS mesajları Serial Port üzerinden gönderir.
        void sendSbusMsg(){
            int* p;
            p = sbus.transmitSerialSBusMessage(sbusChannels);
            for(int i =0; i<16; i++){
                RCLCPP_INFO(this->get_logger()," channels[%d] = %d", i, *(p+i)); //SBUS kanallarındaki değerleri konsola yazdırır.
            }
        }
        
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        int freq = 125;         //RC kumandanın SBUS frekansı yaklaşık 125 Hz'dir. (T = 8 ms)

        float joystick[16];
        int   sbusChannels[16]; //Joystick dizisindeki float değerleri integer olarak sbusChannels dizisine kaydedilir.      

        //interpolasyon
        bool min_button = 0;
        bool max_button = 1;        

        int min_axes = -1;
        int max_axes =  1;

        int min_sbus = 192;  //FC tarafından 1000 olarak algılanır.  
        int max_sbus = 1792; //FC tarafından 2000 olarak algılanır.  

};
        
int main(int argc, char **argv)
{
    if (!sbus.connectSerialPort())
    {
        return 0;
    }
        
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sbus_node>());
    rclcpp::shutdown();
    return 0;
}