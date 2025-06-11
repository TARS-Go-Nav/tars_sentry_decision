#include <string>
#include <vector>
#include <map>
#include <memory>
#include <iostream>
#include <functional>
#include <algorithm>
// XML解析库
#include <tinyxml2.h>

namespace tars_decision {
// 语义点类
class SemanticPoint {
public:
    SemanticPoint(int id, const std::string& code, const std::string& name,  
                    double x, double y)
        : id_(id), code_(code), name_(name), x_(x), y_(y) {}
    
    int id() const { return id_; }
    const std::string& code() const { return code_; }
    const std::string& name() const { return name_; }
    double x() const { return x_; }
    double y() const { return y_; }
    
    std::string toString() const {
        return name_ + " (" + code_ + ", " + std::to_string(x_) + ", " + std::to_string(y_) + ")";
    }
    
private:
    int id_;
    std::string code_;
    std::string name_;
    double x_;
    double y_;
};

// 语义区域类
class SemanticArea {
public:
    SemanticArea(int id, const std::string& code, const std::string& name)
        : id_(id), code_(code), name_(name) {}
    
    void addVertex(double x, double y) {
        vertices_.push_back(std::make_pair(x, y));
    }
    
    void addProperty(const std::string& key, const std::string& value) {
        properties_[key] = value;
    }
    
    int id() const { return id_; }
    const std::string& code() const { return code_; }
    const std::string& name() const { return name_; }
    
    const std::string& getProperty(const std::string& key) const {
        static const std::string empty;
        auto it = properties_.find(key);
        return (it != properties_.end()) ? it->second : empty;
    }
    
    const std::vector<std::pair<double, double>>& getVertices() const {
        return vertices_;
    }
    
    // 判断点是否在区域内（射线法）
    bool contains(double x, double y) const {
        if (vertices_.size() < 3) return false;
        
        bool inside = false;
        size_t n = vertices_.size();
        
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            if (((vertices_[i].second > y) != (vertices_[j].second > y)) &&
                (x < (vertices_[j].first - vertices_[i].first) * (y - vertices_[i].second) / 
                        (vertices_[j].second - vertices_[i].second) + vertices_[i].first)) {
                inside = !inside;
            }
        }
        
        return inside;
    }
    
    std::string toString() const {
        std::string result = name_ + " (" + code_ + ")\nProperties:";
        for (const auto& prop : properties_) {
            result += "\n  " + prop.first + ": " + prop.second;
        }
        result += "\nVertices: " + std::to_string(vertices_.size());
        return result;
    }
    
private:
    int id_;
    std::string code_;
    std::string name_;
    std::vector<std::pair<double, double>> vertices_;
    std::map<std::string, std::string> properties_;
};

// 区域组类
class AreaGroup {
public:
    AreaGroup(int id, const std::string& code, const std::string& name)
        : id_(id), code_(code), name_(name) {}
    
    void addArea(const std::string& area_code) {
        area_codes_.push_back(area_code);
    }
    
    int id() const { return id_; }
    const std::string& code() const { return code_; }
    const std::string& name() const { return name_; }
    const std::vector<std::string>& getAreaCodes() const { return area_codes_; }
    
private:
    int id_;
    std::string code_;
    std::string name_;
    std::vector<std::string> area_codes_;
};

// 语义地图处理器类
class SemanticMapHandler {
public:
    typedef std::shared_ptr<SemanticMapHandler> SharedPtr;
    
    SemanticMapHandler() {}
    
    // 加载语义地图
    bool loadMap(const std::string& map_path) {
        tinyxml2::XMLDocument doc;
        tinyxml2::XMLError result = doc.LoadFile(map_path.c_str());
        
        if (result != tinyxml2::XML_SUCCESS) {
            std::cout << "Failed to load semantic map: " << doc.ErrorStr() << std::endl;
            std::cout << "Error code: " << result << std::endl;
            return false;
        }
        
        tinyxml2::XMLElement* root = doc.RootElement();
        if (!root) {
            std::cout<< "Invalid XML: no root element in semantic map"<< std::endl;
            return false;
        }
        
        // 清除现有数据
        areas_.clear();
        points_.clear();
        area_groups_.clear();
        
        // 解析区域
        tinyxml2::XMLElement* areasElem = root->FirstChildElement("Areas");
        if (areasElem) {
            parseAreas(areasElem);
        }
        
        // 解析点
        tinyxml2::XMLElement* pointsElem = root->FirstChildElement("Points");
        if (pointsElem) {
            parsePoints(pointsElem);
        }
        
        // 解析区域组
        tinyxml2::XMLElement* groupsElem = root->FirstChildElement("AreaGroups");
        if (groupsElem) {
            parseAreaGroups(groupsElem);
        }
        
        std::cout<< "Semantic map loaded successfully"<< std::endl;
                    
        return true;
    }
    
    // 获取点所在的所有区域
    std::vector<std::shared_ptr<SemanticArea>> getAreasContainingPoint(double x, double y) const {
        std::vector<std::shared_ptr<SemanticArea>> result;
        
        for (const auto& pair : areas_) {
            if (pair.second->contains(x, y)) {
                result.push_back(pair.second);
            }
        }
        
        return result;
    }
    
    // 获取区域
    std::shared_ptr<SemanticArea> getArea(const std::string& code) const {
        auto it = areas_.find(code);
        return (it != areas_.end()) ? it->second : nullptr;
    }
    
    // 获取点
    std::shared_ptr<SemanticPoint> getPoint(const std::string& code) const {
        auto it = points_.find(code);
        return (it != points_.end()) ? it->second : nullptr;
    }
    
    // 获取区域组
    std::shared_ptr<AreaGroup> getAreaGroup(const std::string& code) const {
        auto it = area_groups_.find(code);
        return (it != area_groups_.end()) ? it->second : nullptr;
    }
    
    // 判断点是否在指定区域内
    bool isPointInArea(double x, double y, const std::string& area_code) const {
        auto area = getArea(area_code);
        return area ? area->contains(x, y) : false;
    }
    
    // 判断点是否在指定区域组内
    bool isPointInAreaGroup(double x, double y, const std::string& group_code) const {
        auto group = getAreaGroup(group_code);
        if (!group) return false;
        
        for (const auto& area_code : group->getAreaCodes()) {
            if (isPointInArea(x, y, area_code)) {
                return true;
            }
        }
        return false;
    }

    // 检测当前所在区域 传入机器人的xy坐标
    std::string detectCurrentArea(double x, double y) {
        // 查找机器人所在区域
        auto areas = getAreasContainingPoint(x, y);
        
        // 构建区域信息字符串
        std::string area_info;
        if (areas.empty()) {
            area_info = "未知区域";
        } else {
            // 添加高优先级区域和其团队信息
            area_info = areas[0]->name();
            
            // 如果有多个区域，添加次高优先级区域信息
            if (areas.size() > 1) {
                area_info += " - " + areas[1]->name();
            }
        }
        return area_info;
    }
    
private:
    // 解析区域
    void parseAreas(tinyxml2::XMLElement* areasElem) {
        for (tinyxml2::XMLElement* areaElem = areasElem->FirstChildElement("Area"); 
                areaElem; 
                areaElem = areaElem->NextSiblingElement("Area")) {
            
            int id = areaElem->IntAttribute("id");
            const char* code = areaElem->Attribute("code");
            
            if (!code) {
                continue;
            }
            
            // 获取区域名称
            tinyxml2::XMLElement* nameElem = areaElem->FirstChildElement("n");
            std::string name = nameElem ? nameElem->GetText() : "";
            
            // 创建区域对象
            auto area = std::make_shared<SemanticArea>(id, code, name);
            
            // 解析多边形顶点
            tinyxml2::XMLElement* geomElem = areaElem->FirstChildElement("Geometry");
            if (geomElem) {
                tinyxml2::XMLElement* polyElem = geomElem->FirstChildElement("Polygon");
                if (polyElem) {
                    for (tinyxml2::XMLElement* pointElem = polyElem->FirstChildElement("Point"); 
                            pointElem; 
                            pointElem = pointElem->NextSiblingElement("Point")) {                          
                                double x = pointElem->DoubleAttribute("x");
                                double y = pointElem->DoubleAttribute("y");
                                area->addVertex(x, y);
                    }
                }
            }
            
            // 解析属性
            tinyxml2::XMLElement* propsElem = areaElem->FirstChildElement("Properties");
            if (propsElem) {
                for (tinyxml2::XMLElement* propElem = propsElem->FirstChildElement("Property"); 
                        propElem; 
                        propElem = propElem->NextSiblingElement("Property")) {
                        
                    const char* propName = propElem->Attribute("name");
                    const char* propValue = propElem->GetText();
                    
                    if (propName && propValue) {
                        area->addProperty(propName, propValue);
                    }
                }
            }
            
            // 添加到区域映射
            areas_[code] = area;
        }
    }
    
    // 解析点
    void parsePoints(tinyxml2::XMLElement* pointsElem) {
        for (tinyxml2::XMLElement* pointElem = pointsElem->FirstChildElement("Point"); 
                pointElem; 
                pointElem = pointElem->NextSiblingElement("Point")) {
                
            int id = pointElem->IntAttribute("id");
            const char* code = pointElem->Attribute("code");
            
            if (!code) {
                continue;
            }
            
            // 获取点名称
            tinyxml2::XMLElement* nameElem = pointElem->FirstChildElement("n");
            std::string name = nameElem ? nameElem->GetText() : "";
            
            // 获取点坐标
            tinyxml2::XMLElement* posElem = pointElem->FirstChildElement("Position");
            double x = 0.0, y = 0.0;
            if (posElem) {
                x = posElem->DoubleAttribute("x");
                y = posElem->DoubleAttribute("y");
            }
            
            // 创建点对象并添加到映射
            auto point = std::make_shared<SemanticPoint>(id, code, name, x, y);
            points_[code] = point;
        }
    }
    
    // 解析区域组
    void parseAreaGroups(tinyxml2::XMLElement* groupsElem) {
        for (tinyxml2::XMLElement* groupElem = groupsElem->FirstChildElement("AreaGroup"); 
                groupElem; 
                groupElem = groupElem->NextSiblingElement("AreaGroup")) {
                
            int id = groupElem->IntAttribute("id");
            const char* code = groupElem->Attribute("code");
            
            if (!code) {
                std::cout<<"AreaGroup without code attribute skipped"<<std::endl;
                continue;
            }
            
            // 获取组名称
            tinyxml2::XMLElement* nameElem = groupElem->FirstChildElement("n");
            std::string name = nameElem ? nameElem->GetText() : "";
            
            // 创建区域组对象
            auto group = std::make_shared<AreaGroup>(id, code, name);
            
            // 解析包含的区域引用
            tinyxml2::XMLElement* areasElem = groupElem->FirstChildElement("Areas");
            if (areasElem) {
                for (tinyxml2::XMLElement* areaRefElem = areasElem->FirstChildElement("AreaRef"); 
                        areaRefElem; 
                        areaRefElem = areaRefElem->NextSiblingElement("AreaRef")) {
                        
                    const char* areaCode = areaRefElem->GetText();
                    if (areaCode) {
                        group->addArea(areaCode);
                    }
                }
            }
            
            // 添加到区域组映射
            area_groups_[code] = group;
        }
    }
    
    std::map<std::string, std::shared_ptr<SemanticArea>> areas_;
    std::map<std::string, std::shared_ptr<SemanticPoint>> points_;
    std::map<std::string, std::shared_ptr<AreaGroup>> area_groups_;
};
}
