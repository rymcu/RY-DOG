#include <ry_json.h>

void ry_json_test(void)
{
    /**********************************************/
    JsonDocument doc; // 创建变量，添加成员

    doc["sensor"] = "GPS";
    doc["time"] = 12345678;

    // 添加数组
    doc["data"][0] = 12.3;//方法1
    doc["data"][1] = 34.5;

    doc["data"].add(56.7);//方法2
    doc["data"].add(78.9);

    //方法3，效率更高
    JsonArray data_temp = doc["data1"].to<JsonArray>();
    data_temp.add(11.2);
    data_temp.add(22.3);
    
    //Serialization:一种将对象转换为字节流的机制
    char output[256];
    serializeJson(doc,output);
    //Deserialization:与Serialization相反
    Serial.println(output);
}