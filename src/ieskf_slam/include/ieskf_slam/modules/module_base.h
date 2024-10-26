#pragma once
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>//用于读取配置文件


namespace IESKFSlam
{
  class ModuleBase
  {
  private:
    YAML::Node config_node;
    std::string name;
  protected:
  ModuleBase(const std::string&config_path,const std::string&prefix, const std::string &module_name = "default"){
    name = module_name;
    if(config_path !=""){
      try{
          config_node = YAML::LoadFile(config_path);
      }catch (YAML::Exception &e){//YAML::Exception 指明了这个 catch 块将要捕获的异常类型。这意味着只有 YAML::Exception 类型（或其派生类型）的异常才会被这个 catch 块处理。
        // &e 表示通过引用捕获异常。这是一种高效的捕获方式，可以避免异常对象的复制
        std::cout<<e.msg<<std::endl;
      }

      if(prefix !=""&&config_node[prefix]) config_node = config_node[prefix];
    }


    }

    template<typename T>
    void readParam(const std::string& key,T& val,T default_value){
    if(config_node[key]){
      val = config_node[key].as<T>();//用于将 YAML 节点转换为程序员指定的类型 T
    }else{
      val = default_value;
    }
    }
  };
}


