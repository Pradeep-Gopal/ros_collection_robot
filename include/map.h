class OrderManager{

private:

    std::vector<char>cubes_;
    std::vector<char>order_;
    ros::NodeHandle nh_;
    ros::Publisher order_pub_;
    int total_cubes_ = 8;
    int order_size_ = 4;

public:

    void generateOrder();
    void spawnCubes();

    // getters

    int getTotalCubes();
    int getOrderSize();
    std::vector<char> getOrder();
    std::vector<char> getCubes();
};
