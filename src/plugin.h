namespace plugin {

// helpers for constructor overloading (deprecated)

struct dummy_func_t { }; // Dummy func tag type
static const dummy_func_t dummy; // Dummy func tag object

// meta template for functions
#if (defined(__GNUC__) || defined(__GNUG__) || defined(__clang__))
template<typename Func>
struct meta;
#elif (defined(_MSC_VER))
template<typename FuncType, FuncType Func>
struct meta;
#else

#endif

// meta templates for constructors, destructors and new/delete operators

template<typename ClassType, typename OverloadedDecl = void()>
struct ctor_meta;

template<typename ClassType>
struct dtor_meta;

template<typename ClassType>
struct del_dtor_meta;

template<typename ClassType, typename OverloadedDecl = void *(unsigned int)>
struct op_new_meta;

template<typename ClassType, typename OverloadedDecl = void *(unsigned int)>
struct op_new_array_meta;

template<typename ClassType, typename OverloadedDecl = void(void *)>
struct op_delete_meta;

template<typename ClassType, typename OverloadedDecl = void *(unsigned int)>
struct op_delete_array_meta;

// vtable description
template<typename ClassType>
struct vtable_meta;

// stack_object

template <typename T>
struct stack_object {
private:
    T object;
public:
    template<typename ...ArgTypes>
    stack_object(ArgTypes... args) : object(args...) {}
    T *operator->() { return &object; }
    T &get_object() { return object; }
};

template <typename T>
struct stack_object_no_default {
    stack_object_no_default() {}
    stack_object_no_default(stack_object_no_default &&) = delete;
    stack_object_no_default &operator=(stack_object_no_default &&) = delete;
protected:
    char objBuff[sizeof(T)];
public:
    T *operator->() { return reinterpret_cast<T *>(objBuff); }
    T &get_object() { return *reinterpret_cast<T *>(objBuff); }
};

// custom new/delete

template <typename ClassType, typename... ArgTypes>
ClassType *operator_new(ArgTypes... args) {
    return new ClassType(args...);
}

template <typename ClassType>
ClassType *operator_new_array(size_t size) {
    return new ClassType[size];
}

template <typename ClassType>
void operator_delete(ClassType *data) {
    delete data;
}

template <typename ClassType>
void operator_delete_array(ClassType *data) {
    delete[] data;
}

// multi-version address list
template<int... Addresses>
struct MvAddresses {};

// Gets the virtual method table from the object @self
inline void** GetVMT(const void* self) {
    return *(void***)(self);
}

// Gets the virtual method from @self in the table index @index 
inline void* GetVMT(const void* self, size_t index) {
    return GetVMT(self)[index];
}

template <unsigned int address, typename... Args>
void Call(Args... args) {
    reinterpret_cast<void(__cdecl *)(Args...)>(address)(args...);
}

template <typename Ret, unsigned int address, typename... Args>
Ret CallAndReturn(Args... args) {
    return reinterpret_cast<Ret(__cdecl *)(Args...)>(address)(args...);
}

template <unsigned int address, typename C, typename... Args>
void CallMethod(C _this, Args... args) {
    reinterpret_cast<void(__thiscall *)(C, Args...)>(address)(_this, args...);
}

template <typename Ret, unsigned int address, typename C, typename... Args>
Ret CallMethodAndReturn(C _this, Args... args) {
    return reinterpret_cast<Ret(__thiscall *)(C, Args...)>(address)(_this, args...);
}

template <unsigned int tableIndex, typename C, typename... Args>
void CallVirtualMethod(C _this, Args... args) {
    reinterpret_cast<void(__thiscall *)(C, Args...)>((*reinterpret_cast<void ***>(_this))[tableIndex])(_this, args...);
}

template <typename Ret, unsigned int tableIndex, typename C, typename... Args>
Ret CallVirtualMethodAndReturn(C _this, Args... args) {
    return reinterpret_cast<Ret(__thiscall *)(C, Args...)>((*reinterpret_cast<void ***>(_this))[tableIndex])(_this, args...);
}

template <typename... Args>
void CallDyn(unsigned int address, Args... args) {
    reinterpret_cast<void(__cdecl *)(Args...)>(address)(args...);
}

template <typename Ret, typename... Args>
Ret CallAndReturnDyn(unsigned int address, Args... args) {
    return reinterpret_cast<Ret(__cdecl *)(Args...)>(address)(args...);
}

template <typename C, typename... Args>
void CallMethodDyn(unsigned int address, C _this, Args... args) {
    reinterpret_cast<void(__thiscall *)(C, Args...)>(address)(_this, args...);
}

template <typename Ret, typename C, typename... Args>
Ret CallMethodAndReturnDyn(unsigned int address, C _this, Args... args) {
    return reinterpret_cast<Ret(__thiscall *)(C, Args...)>(address)(_this, args...);
}

template <typename... Args>
void CallDynGlobal(unsigned int address, Args... args) {
    reinterpret_cast<void(__cdecl *)(Args...)>(address)(args...);
}

template <typename Ret, typename... Args>
Ret CallAndReturnDynGlobal(unsigned int address, Args... args) {
    return reinterpret_cast<Ret(__cdecl *)(Args...)>(address)(args...);
}

template <typename C, typename... Args>
void CallMethodDynGlobal(unsigned int address, C _this, Args... args) {
    reinterpret_cast<void(__thiscall *)(C, Args...)>(address)(_this, args...);
}

template <typename Ret, typename C, typename... Args>
Ret CallMethodAndReturnDynGlobal(unsigned int address, C _this, Args... args) {
    return reinterpret_cast<Ret(__thiscall *)(C, Args...)>(address)(_this, args...);
}

};