#ifndef DSRTYPETRAIT_H
#define DSRTYPETRAIT_H

// Type codification for encoder
template<typename T> struct DSRTypeTrait;
template<> struct DSRTypeTrait<std::string>             { static constexpr const char* code = "s"; };
template<> struct DSRTypeTrait<int32_t>                 { static constexpr const char* code = "i"; };
template<> struct DSRTypeTrait<float>                   { static constexpr const char* code = "f"; };
template<> struct DSRTypeTrait<std::vector<float>>      { static constexpr const char* code = "vf"; };
template<> struct DSRTypeTrait<bool>                    { static constexpr const char* code = "b"; };
template<> struct DSRTypeTrait<std::vector<uint8_t>>    { static constexpr const char* code = "vc"; };
template<> struct DSRTypeTrait<uint32_t>                { static constexpr const char* code = "ui"; };
template<> struct DSRTypeTrait<uint64_t>                { static constexpr const char* code = "ui64"; };
template<> struct DSRTypeTrait<double>                  { static constexpr const char* code = "d"; };
template<> struct DSRTypeTrait<std::vector<uint64_t>>   { static constexpr const char* code = "vui64"; };
template<> struct DSRTypeTrait<std::array<float, 2>>    { static constexpr const char* code = "v2"; };
template<> struct DSRTypeTrait<std::array<float, 3>>    { static constexpr const char* code = "v3"; };
template<> struct DSRTypeTrait<std::array<float, 4>>    { static constexpr const char* code = "v4"; };
template<> struct DSRTypeTrait<std::array<float, 6>>    { static constexpr const char* code = "v6"; };

// Attributes types
namespace DSRAttributeTypes {
    using AttributeType = std::variant<
        std::string, 
        int32_t, 
        float,
        std::vector<float>,
        bool, 
        std::vector<uint8_t>,
        uint32_t, 
        uint64_t, 
        double, 
        std::vector<uint64_t>,
        std::array<float, 2>, 
        std::array<float, 3>,
        std::array<float, 4>, 
        std::array<float, 6>
    >;
}

// Special characters for encoder string delimitation
namespace DSRSpecialChars {
	constexpr char SLOT = '#';
	constexpr char ATT_NAME = '$';
	constexpr char ATT_TYPE = ':';
	constexpr char ATT_VAL = '%';
	constexpr char K_DIV = '@';
	
	constexpr const char* K = "K";
	constexpr const char* MN = "MN";
	constexpr const char* MNA = "MNA";
	constexpr const char* ME = "ME";
	constexpr const char* MEA = "MEA";
	constexpr const char* DN = "DN";
	constexpr const char* DE = "DE";
}

// Attributes names for encoder
namespace DSRAttributeNames {
    constexpr const char* ID = "id";      // node id
    constexpr const char* IDF = "idf";    // edge from id
    constexpr const char* IDT = "idt";    // edge to id
    constexpr const char* NODE_NAME = "name";  // node name
    constexpr const char* TYPE = "type";  // node/edge type
}


#endif
