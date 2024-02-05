/*
    A template class for low-pass filtering. 
*/
template<typename T>
class LowPassFilter {
private:
    double alpha;
    T filtered_value;

public:
    LowPassFilter(double alpha,const T& initial_value)
        : alpha(alpha) {
            //Initialize with zeros
            filtered_value = initial_value;
        }

    T filter(const T& input) {
        // Apply the low-pass filter formula
        filtered_value = alpha * input + (1 - alpha) * filtered_value;
        return filtered_value;
    }
};

