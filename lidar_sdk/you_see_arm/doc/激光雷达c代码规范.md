---
title: "激光雷达 c 代码规范"
output:
  word_document:
    path: 激光雷达 c 代码规范.docx
    toc: true
    toc_depth: 6
    number_sections: true
export_on_save:
  pandoc: true
---

| 修改日期           | 描述                        | 修改人 |
| ------------------ | --------------------------- | ------ |
| 2024 年 5 月 11 日 | 创建激光雷达 c 代码规范初版 | Hu     |

# 1 规范目标

规范目标参考了
**[LLVM C 规范](https://llvm.org/docs/CodingStandards.html)**、
**[Linux 内核 c 代码规范](https://www.kernel.org/doc/html/latest/translations/zh_CN/process/coding-style.html)**、
**[华为 c 代码规范归档版](https://gitee.com/openharmony/docs/blob/master/zh-cn/contribute/OpenHarmony-c-coding-style-guide.md)**、
**[华为 c 代码精华版](https://blog.csdn.net/m0_38106923/article/details/105042594)**

本文档综合以上规范在日常开发中的常用部分，并结合工程实际应用。重要的地方会指明来处，想要了解更深入、详细的信息，请到参考文档指定位置查看，如果有与参考文档矛盾的地方，以本文档为准。

## 1.1 清晰

代码风格因人而异，希望能够做到像 Linux 内核那样简单清晰，避免给人误读的可能

> 目前软件维护期成本占整个生命周期成本的 40%~90%。根据业界经验，维护期变更代码的成本，小型系统是开发期的 5 倍，大型系统（100 万行代码以上）可以达到 100 倍。业界的调查指出，开发组平均大约一半的人力用于弥补过去的错误，而不是添加新的功能来帮助公司提高竞争力。 -- 引用自 华为 c 代码精华版 **[华为 c 代码精华版](https://blog.csdn.net/m0_38106923/article/details/105042594)**

## 1.2 简洁

代码越多，出错的概率越大，一个代码模块尽量只干一件事情。废弃的代码 (没有被调用的函数和全局变量) 要及时清除，重复代码应该尽可能提炼成函数。

## 1.3 风格一致

激光雷达项目的代码风格，请按照本文档建议保持一致。

# 2 头文件

本章参考 **[华为 c 代码规范归档版](https://gitee.com/openharmony/docs/blob/master/zh-cn/contribute/OpenHarmony-c-coding-style-guide.md)** 第四章 头文件
对于 C 语言来说，头文件的设计体现了大部分的系统设计。
正确使用头文件可使代码在可读性、文件大小和编译构建性能上大为改观。

本章从编程规范的角度总结了一些方法，可用于帮助合理规划头文件。

## 2.1 头文件职责

头文件是模块或文件的对外接口。
头文件中适合放置接口的声明，不允许放置实现（内联函数除外）。
头文件应当职责单一。头文件过于复杂，依赖过于复杂还是导致编译时间过长的主要原因。

## 2.2 建议每一个.c 文件都应该有相应的.h 文件

用于声明需要对外公开的接口。

通常情况下，每个.c 文件都有一个相应的.h（并不一定同名），用于放置对外提供的函数声明、宏定义、类型定义等。
如果一个.c 文件不需要对外公布任何接口，则其就不应当存在。

例外：程序的入口（如 main 函数所在的文件），单元测试代码，动态库代码。

示例:
foo.h 内容

```cpp
#ifndef FOO_H
#define FOO_H

int foo(void);  // Good：头文件中声明对外接口

#endif
```

foo.c 内容

```cpp
static void Bar(void);  // Good: 对内函数的声明放在.c 文件的头部，并声明为 static 限制其作用域

void foo(void)
{
    bar();
}

static void bar(void)
{
    // Do something;
}
```

有些产品中，习惯一个.c 文件对应两个.h 文件，一个用于存放对外公开的接口，一个用于存放内部需要用到的定义、声明等，以控制.c 文件的代码行数。
不提倡这种风格，产生这种风格的根源在于.c 过大，应当首先考虑拆分.c 文件。
另外，一旦把私有定义、声明放到独立的头文件中，就无法从技术上避免别人包含。

本规则反过来并不一定成立。比如：
有些特别简单的头文件，如命令 ID 定义头文件，不需要有对应的.c 存在。
同一套接口协议下，有多个实例，由于接口相同且稳定，所以允许出现一个.h 对应多个.c 文件。

## 2.3 建议头文件的扩展名只使用.h

不使用非习惯用法的扩展名，如.inc。

有些产品中使用了 .inc 作为头文件扩展名，这不符合 C 语言的习惯用法。在使用 .inc 作为头文件扩展名的产品，习惯上用于标识此头文件为私有头文件。但是从产品的实际代码来看，这一条并没有被遵守，一个 .inc 文件被多个 .c 包含。本规范不提倡将私有定义单独放在头文件中，具体见 2.2。

## 2.4 头文件依赖

头文件包含是一种依赖关系，头文件应向稳定的方向包含。
一般来说，应当让不稳定的模块依赖稳定的模块，从而当不稳定的模块发生变化时，不会影响（编译）稳定的模块。

依赖的方向应该是：产品依赖于平台，平台依赖于标准库。

除了不稳定的模块依赖于稳定的模块外，更好的方式是每个模块都依赖于接口，这样任何一个模块的内部实现更改都不需要重新编译另外一个模块。
在这里，假设接口本身是最稳定的。

### 2.4.1 禁止头文件循环依赖

头文件循环依赖，指 a.h 包含 b.h，b.h 包含 c.h，c.h 包含 a.h，导致任何一个头文件修改，都导致所有包含了 a.h/b.h/c.h 的代码全部重新编译一遍。
而如果是单向依赖，如 a.h 包含 b.h，b.h 包含 c.h，而 c.h 不包含任何头文件，则修改 a.h 不会导致包含了 b.h/c.h 的源代码重新编译。

头文件循环依赖直接体现了架构设计上的不合理，可通过架构优化来避免。

### 2.4.2 头文件必须编写#define 保护，防止重复包含

为防止头文件被多重包含，所有头文件都应当使用 #define 作为包含保护；不要使用 #pragma once

定义包含保护符时，应该遵守如下规则：

- 保护符使用唯一名称；统一使用子系统名*部件名*文件名的定义规则。
- 不要在受保护部分的前后放置代码或者注释，文件头注释除外。
  假定 util 子系统 timer 部件的 timer.h，其目录为 timer/include/timer.h。其保护符若使用 'TIME_H' 很容易不唯一，按规则定义如：

```cpp
#ifndef UTIL_TIMER_TIMER_H
#define UTIL_TIMER_TIMER_H

...

#endif // UTIL_TIMER_TIMER_H
```

### 2.4.3 禁止通过声明的方式引用外部函数接口、变量

只能通过包含头文件的方式使用其他模块或文件提供的接口。
通过 extern 声明的方式使用外部函数接口、变量，容易在外部接口改变时可能导致声明和定义不一致。
同时这种隐式依赖，容易导致架构腐化。

不符合规范的案例：
a.c 内容

```cpp
extern int foo(void);   // Bad: 通过 extern 的方式引用外部函数

void Bar(void)
{
    int i = foo();  // 这里使用了外部接口 Foo
    ...
}
```

应该改为：
a.c 内容

```cpp
#include "b.h"      // Good: 通过包含头文件的方式使用其他.c 提供的接口

void bar(void)
{
    int i = foo();
    ...
}
```

b.h 内容

```cpp
int foo(void);
```

b.c 内容

```cpp
int foo(void)
{
    // Do something
}
```

例外，有些场景需要引用其内部函数，但并不想侵入代码时，可以 extern 声明方式引用。
如：
针对某一内部函数进行单元测试时，可以通过 extern 声明来引用被测函数；
当需要对某一函数进行打桩、打补丁处理时，允许 extern 声明该函数。

### 2.4.4 禁止在 extern "C" 中包含头文件

在 extern "C" 中包含头文件，有可能会导致 extern "C" 嵌套，部分编译器对 extern "C" 嵌套层次有限制，嵌套层次太多会编译错误。

extern "C" 通常出现在 C，C++ 混合编程的情况下，在 extern "C" 中包含头文件，可能会导致被包含头文件的原有意图遭到破坏，比如链接规范被不正确地更改。

示例，存在 a.h 和 b.h 两个头文件：
a.h 内容

```cpp
...
#ifdef __cplusplus
void foo(int);
#define A(value) foo(value)
#else
void a(int)
#endif
```

b.h 内容

```cpp
...
#ifdef __cplusplus
extern "C" {
#endif

#include "a.h"
void b(void);

#ifdef __cplusplus
}
#endif
```

使用 C++ 预处理器展开 b.h，将会得到

```cpp
extern "C" {
    void foo(int);
    void b(void);
}
```

按照 a.h 作者的本意，函数 Foo 是一个 C++ 自由函数，其链接规范为 "C++"。但在 b.h 中，由于 #include "a.h" 被放到了 extern "C" 的内部，函数 Foo 的链接规范被不正确地更改了。

例外：如果在 C++ 编译环境中，想引用纯 C 的头文件，这些 C 头文件并没有 extern "C" 修饰。非侵入式的做法是，在 extern "C" 中去包含 C 头文件。

# 3 命名

本章参考
**[Linux 内核 c 代码规范](https://www.kernel.org/doc/html/latest/translations/zh_CN/process/coding-style.html)** 第四章 命名、
**[华为 c 代码规范归档版](https://gitee.com/openharmony/docs/blob/master/zh-cn/contribute/OpenHarmony-c-coding-style-guide.md)** 第一章 命名

命名包括文件、函数、变量、类型、宏等命名。

命名被认为是软件开发过程中最困难，也是最重要的事情。
标识符的命名要清晰、明了，有明确含义，符合阅读习惯，容易理解。

C 是一个简朴的语言，你的命名也应该这样。C 程序员不使用类似 ThisVariableIsATemporaryCounter 这样华丽的名字。C 程序员会 称那个变量为 tmp，这样写起来会更容易，而且至少不会令其难于理解。

## 3.1 作用域越大，命名越准确

虽然混用大小写的名字是不提倡使用的，但是全局变量还是需要一个具描述性的 名字。称一个全局函数为 foo 是一个难以饶恕的错误。

全局变量 (只有当你 真正 需要它们的时候再用它) 需要有一个具描述性的名字，就 像全局函数。如果你有一个可以计算活动用户数量的函数，你应该叫它 count_active_users() 或者类似的名字，你不应该叫它 cntuser()

C 与 C++ 不同，没有名字空间，没有类，所以全局作用域下的标识符命名要考虑不要冲突。
对于全局函数、全局变量、宏、类型名、枚举名的命名，应当精确描述并全局唯一。

```cpp
int get_count(void); // Bad: 描述不精确
int get_active_connect_count(void);  // Good
```

## 3.2 文件命名全部使用小写字符

文件名命名只允许使用小写字母、数字以及下划线 (\_)。
文件名应尽量简短、准确、无二义性。
不大小写混用的原因是，不同系统对文件名大小写处理会不同（如 MicroSoft 的 DOS / Windows 系统不区分大小写，但是 Unix / Linux / Mac 系统则默认区分）。

好的命名举例：

- dhcp_user_log.c

## 3.3 函数、类型全部使用小写字符

类型包括结构体、联合体、枚举类型名，
好的命名举例

```cpp
int wakeup_secondary(unsigned int cpu); // good 函数命名

enum ftrace_dump_mode { // good 枚举名全小写，用下划线连接
    DUMP_NONE, // good 枚举项全大写，用下划线连接
    DUMP_ALL,
    DUMP_ORIG,
};

struct block_list { // good 结构体名全小写，用下划线连接
    char *txt;
    int page_num; // good 结构项全小写，用下划线连接
    int allocator;
};

union acpi_object in_params[4]; // good 联合体全小写，用下划线连接
```

## 3.4 变量命名

### 3.4.1 全局变量应增加 'g\_' 前缀

全局变量应当尽量少使用，使用时应特别注意，所以加上前缀用于视觉上的突出，促使开发人员对这些变量的使用更加小心。
全局静态变量命名与全局变量相同，函数内的静态变量命名与普通局部变量相同。

```cpp
int g_active_connect_count; // good 全局变量加 g_前缀
```

### 3.4.2 本地变量名应该简短，而且能够表达相关的含义

如果你有一些随机的整数型的循环计 数器，它应该被称为 i。叫它 loop_counter 并无益处，如果它没有被误解的 可能的话。类似的，tmp 可以用来称呼任意类型的临时变量。

## 3.5 宏、常量、枚举命名

宏、枚举值采用全大写，下划线连接的格式。
常量推荐采用全大写，下划线连接风格。作为全局变量，也可以保持与普通全局变量命名风格相同。
这里常量如前文定义，是指基本数据类型、枚举、字符串类型的全局 const 变量。

函数式宏，命名风格，采用全大写，下划线连接风格。
宏举例：

```cpp
#define PI 3.14
#define MAX(a, b)   (((a) < (b)) ? (b) : (a))
```

常量举例：

```cpp
const int VERSION = 200;    // OK.

const enum color DEFAULT_COLOR = BLUE; // OK.

const char PATH_SEP = '/';  // OK.

const char * const GREETINGS = "Hello, World!"; // OK.
```

## 3.6 避免函数式宏中的临时变量命名污染外部作用域

首先，定义函数式宏前，应考虑能否定义为函数。如果可以则不要定义为函数式宏。

当函数式宏需要定义局部变量时，为了防止跟外部函数中的局部变量有命名冲突。

后置下双划线，是一种解决方案。例：

```cpp
#define SWAP_INT(a, b) do { \
    int tmp__ = a; \
    a = b; \
    b = tmp__; \
} while (0)
```

## 3.7 使用 Codelf 辅助命名

可以使用 vscode 的 Codelf 插件来生成命名，一般会生成多个，选择符合本章命名规范的名称。

# 4 排版格式

本章参考
**[Linux 内核 c 代码规范](https://www.kernel.org/doc/html/latest/translations/zh_CN/process/coding-style.html)** 第一章 缩进、第三章 大括号和空格的放置、
**[华为 c 代码规范归档版](https://gitee.com/openharmony/docs/blob/master/zh-cn/contribute/OpenHarmony-c-coding-style-guide.md)** 第二章 排版格式

## 4.1 行宽不超过 120 个字符

代码行宽不宜过长，否则不利于阅读。
控制行宽长度可以间接的引导开发去缩短函数、变量的命名，减少嵌套的层数，提升代码可读性。
要求每行字符数不要超过 120 个；除非超过 120 能显著增加可读性，并且不会隐藏信息。
虽然现代显示器分辨率已经很高，但是行宽过长，反而提高了阅读理解的难度；跟本规范提倡的“清晰”、“简洁”原则相背。

如下场景不宜换行，可以例外：

- 换行会导致内容截断，无法被方便查找 (grep) 的字符串，如命令行或 URL 等等。包含这些内容的代码或注释，可以适当例外。
- #include / #error 语句可以超出行宽要求，但是也需要尽量避免。

## 4.2 缩进

每次缩进 4 个字符，请使用制表符来缩进，不要使用空格，不要在行尾留空格。vscode 查看缩进，推荐使用插件 indent-rainbow

在 switch 语句中消除多级缩进的首选的方式是让 switch 和从属于它的 case 标签对齐于同一列，而不要再次缩进 case 标签。比如：

```cpp
switch (suffix) {
case 'G':
case 'g':
    mem <<= 30;
    break;
case 'M':
case 'm':
    mem <<= 20;
    break;
case 'K':
case 'k':
    mem <<= 10;
    fallthrough;
default:
    break;
}
```

## 4.3 不把多条语句放在同一行

也不要在一行里放多个赋值语句，不要使用逗号避免大括号

```cpp
if (condition)
    do_this(), do_that();
```

使用大括号包裹多语句：

```cpp
if (condition) {
        do_this();
        do_that();
}
```

if/else/else if 不可以写在同一行，条件语句中，若有多个分支，应该写在不同行。

如下是正确的写法：

```cpp
if (someConditions) {
    ...
} else {    // good: else 与 if 在不同行
    ...
}
```

## 4.4 大括号

## 4.4.1 大括号位置

首选的方式，就像 Kernighan 和 Ritchie（c 语言作者，即 K&R 风格）展示 给我们的，是把起始大括号放在行尾，而把结束大括号放在行首。

所以就是这样：

```cpp
if (x is true) {
    we do y
}
```

这适用于所有的非函数语句块 (if, switch, for, while, do)。比如：

```cpp
switch (action) {
case KOBJ_ADD:
    return "add";
case KOBJ_REMOVE:
    return "remove";
case KOBJ_CHANGE:
    return "change";
default:
    return NULL;
}
```

不过，有一个例外，那就是函数：函数的起始大括号放置于下一行的开头，所以：

```cpp
int function(int x)
{
    body of function
}
```

## 4.4.2 条件语句必须使用大括号

虽然在 Linux 的内核代码中，只有一行并且没有大括号的语句随处可见，例如：

```cpp
if (ctx->flags & IORING_SETUP_CQE32)
    cq_shift = 1;
if (ctx->flags & IORING_SETUP_SQE128)
    sq_shift = 1;
```

但是我们还是强制要求，条件语句必须使用大括号。原因如下：

- 代码逻辑直观，易读，与其他多行的条件语句保持一致。
- 在已有条件语句代码上增加新代码时不容易出错。
- 对于在条件语句中使用函数式宏时，没有大括号保护容易出错（如果宏定义时遗漏了大括号）。

例如：

```cpp
if (object_is_not_exist) {         // good：单行条件语句也加大括号
    return create_new_object();
}
```

## 4.5 空格

空格的使用原则，参考 liunux 内核。它的空格使用方式 (主要) 取决于它是用于函数还是关键字。(大多数) 关键字 后要加一个空格。值得注意的例外是 sizeof, typeof, alignof 和 **attribute**，这 些关键字某些程度上看起来更像函数 (它们在 Linux 里也常常伴随小括号而使用，尽管 在 C 里这样的小括号不是必需的，就像 struct fileinfo info; 声明过后的 sizeof info)

所以在这些关键字之后放一个空格：

```cpp
if, switch, case, for, do, while
```

但是不要在 sizeof, typeof, alignof 或者 **attribute** 这些关键字之后放空格。应该这样使用，例如：

```cpp
s = sizeof(struct file);
```

不要在小括号里的表达式两侧加空格。这是一个 **反例** ：

```cpp
s = sizeof( struct file );
```

当声明指针类型或者返回指针类型的函数时， \* 的首选使用方式是使之靠近变量名 或者函数名，而不是靠近类型名。例子：

```cpp
char *linux_banner;
unsigned long long memparse(char *ptr, char **retptr);
char *match_strdup(substring_t *s);
```

在大多数二元和三元操作符两侧使用一个空格，例如下面所有这些操作符：

```cpp
=  +  -  <  >  *  /  %  |  &  ^  <=  >=  ==  !=  ?  :
```

但是一元操作符后不要加空格：

```cpp
&  *  +  -  ~  !  sizeof  typeof  alignof  __attribute__  defined
```

后缀自加和自减一元操作符前不加空格：

```cpp
++  --
```

前缀自加和自减一元操作符后不加空格：

```cpp
++  --
```

. 和 -> 结构体成员操作符前后不加空格。

不要在行尾留空白。有些可以自动缩进的编辑器会在新行的行首加入适量的空白，然后 你就可以直接在那一行输入代码。不过假如你最后没有在那一行输入代码，有些编辑器 就不会移除已经加入的空白，就像你故意留下一个只有空白的行。包含行尾空白的行就 这样产生了。

当 git 发现补丁包含了行尾空白的时候会警告你，并且可以应你的要求去掉行尾空白；不过如果你是正在打一系列补丁，这样做会导致后面的补丁失败，因为你改变了补丁的 上下文。

# 5 注释

注释参考
**[Linux 内核 c 代码规范](https://www.kernel.org/doc/html/latest/translations/zh_CN/process/coding-style.html)** 第八章 注释、
**[编写 kernel-doc 注释](https://www.kernel.org/doc/html/latest/translations/zh_CN/doc-guide/kernel-doc.html)**、
**[华为 c 代码规范归档版](https://gitee.com/openharmony/docs/blob/master/zh-cn/contribute/OpenHarmony-c-coding-style-guide.md)** 第三章 注释

在中文环境下编程，很遗憾，注释使用中文。
注释是好的，不过有过度注释的危险。永远不要在注释里解释你的代码是如何运作的。更好的做法是让别人一看你的代码就可以明白，解释写的很差的代码是浪费时间。

**一般来说你用注释告诉别人你的代码做了什么，而不是怎么做的**。也请你不要把 注释放在一个函数体内部：如果函数复杂到你需要独立的注释其中的一部分，你很可能 需要考虑函数编写得是否满足函数章节的要求。你可以做一些小注释来注明或警告某些很聪明 (或者槽糕) 的 做法，但不要加太多。你应该做的，是把注释放在函数的头部，告诉人们它做了什么，也可以加上它做这些事情的原因。

## 5.1 函数和函数式宏的注释的格式

```cpp
/**
 * 函数名 () - 函数简要说明.
 * @参数 1: 描述第一个参数.
 * @参数 2: 描述第二个参数.
 *        可以为参数提供一段
 *        多行描述.
 *
 * 更详细的描述，进一步讨论函数 函数名 (), 这可能对使用或修改它的人有用.
 * 以空注释行开始，内部可以包含空注释行.
 *
 * 详细描述可以有多个段落.
 *
 * Context: 描述函数是否可以休眠，它需要、释放或期望持有什么锁.
 *          可以写多行.
 * Return: 描述函数返回值.
 *
 * 返回值描述也可以有多个段落，
 * 并且应该放在注释块的末尾.
 */
```

例如，下面示例来自内核源码，注释内容为英文，这里主要展示一下格式

```cpp
/**
 * arch_elf_pt_proc() - check a PT_LOPROC..PT_HIPROC ELF program header
 * @ehdr:	The main ELF header
 * @phdr:	The program header to check
 * @elf:	The open ELF file
 * @is_interp:	True if the phdr is from the interpreter of the ELF being
 *		loaded, else false.
 * @state:	Architecture-specific state preserved throughout the process
 *		of loading the ELF.
 *
 * Inspects the program header phdr to validate its correctness and/or
 * suitability for the system. Called once per ELF program header in the
 * range PT_LOPROC to PT_HIPROC, for both the ELF being loaded and its
 * interpreter.
 *
 * Return: Zero to proceed with the ELF load, non-zero to fail the ELF load
 *         with that return code.
 */
static inline int arch_elf_pt_proc(struct elfhdr *ehdr,
				   struct elf_phdr *phdr,
				   struct file *elf, bool is_interp,
				   struct arch_elf_state *state)
{
	/* Dummy implementation, always proceed */
	return 0;
}

```

函数名后面的简短描述可以跨多行，并以参数描述、空注释行或注释块结尾结束。

### 5.1.1 函数参数

每个函数参数都应该按照顺序描述，紧跟在函数简要说明之后。不要在函数描述和参数 之间，也不要在参数之间留空。
如果 @参数 描述有多行，则说明的续行应该从上一行的同一列开始：

```cpp
* @参数: 较长说明
*        的续行
或:

* @参数:
*         较长说明
*         的续行
```

如果函数的参数数目可变，则需对其进行描述：

```cpp
* @...: 描述
```

### 5.1.2 函数上下文

可调用函数的上下文应该在 Context 节中描述。此节应该包括函数是休眠的还是 可以从中断上下文调用的，以及它需要什么锁、释放什么锁和期望它的调用者持有什么 锁。

例如：

```cpp
* Context: Any context.
* Context: Any context. Takes and releases the RCU lock.
* Context: Any context. Expects <lock> to be held by caller.
* Context: Process context. May sleep if @gfp flags permit.
* Context: Process context. Takes and releases <mutex>.
* Context: Softirq or process context. Takes and releases <lock>, BH-safe.
* Context: Interrupt context.
```

### 5.1.3 返回值

如有返回值，应在 Return 节中描述。

```cpp
* Return:
* * 0             - OK to runtime suspend the device
* * -EBUSY        - Device should not be runtime suspended
```

## 5.2 结构体、共用体、枚举类型

结构体（struct）、共用体（union）、枚举（enum）类型注释的一般格式为：

```cpp
/*
 * struct 结构体名 - 简要描述.
 * @成员 1: 成员 1 描述.
 * @成员 2: 成员 2 描述.
 *           可以为成员提供
 *           多行描述.
 *
 * 结构体的描述.
 */
```

可以用 union 或 enum 替换上面示例中的 struct，以描述共用体或枚举。成员 用于表示枚举中的元素或共用体成员。

结构体名称后面的简要说明可以跨多行，并以成员说明、空白注释行或注释块结尾结束。

成员
结构体、共用体和枚举的成员应以与函数参数相同的方式记录；它们后紧跟简短的描述，并且为多行。

在结构体或共用体描述中，可以使用 private: 和 public: 注释标签。

private: 和 public: 标签必须紧跟在 /_ 注释标记之后。可以选择是否 在 : 和 _/ 结束标记之间包含注释。

例子：

```cpp
/*
 * struct 张三 - 简短描述
 * @a: 第一个成员
 * @b: 第二个成员
 * @d: 第三个成员
 *
 * 详细描述
 */
struct 张三 {
    int a;
    int b;
/* private: 仅内部使用 */
    int c;
/* public: 下一个是公有的 */
    int d;
};
```

嵌套的结构体/共用体
嵌套的结构体/共用体可像这样记录：

```cpp
/*
 * struct nested_foobar - a struct with nested unions and structs
 * @memb1: first member of anonymous union/anonymous struct
 * @memb2: second member of anonymous union/anonymous struct
 * @memb3: third member of anonymous union/anonymous struct
 * @memb4: fourth member of anonymous union/anonymous struct
 * @bar: non-anonymous union
 * @bar.st1: struct st1 inside @bar
 * @bar.st2: struct st2 inside @bar
 * @bar.st1.memb1: first member of struct st1 on union bar
 * @bar.st1.memb2: second member of struct st1 on union bar
 * @bar.st2.memb1: first member of struct st2 on union bar
 * @bar.st2.memb2: second member of struct st2 on union bar
 */
struct nested_foobar {
  /* Anonymous union/struct*/
  union {
    struct {
      int memb1;
      int memb2;
    };
    struct {
      void *memb3;
      int memb4;
    };
  };
  union {
    struct {
      int memb1;
      int memb2;
    } st1;
    struct {
      void *memb1;
      int memb2;
    } st2;
  } bar;
};
```

在记录嵌套结构体或共用体时，如果结构体/共用体 张三 已命名，则其中 的成员 李四 应记录为 @张三。李四：

当嵌套结构体/共用体是匿名的时，其中的成员 李四 应记录为 @李四：

行间注释文档
结构成员也可在定义时以行间注释形式记录。有两种样式，一种是单行注释，其中开始 /\*_ 和结束 _/ 位于同一行；另一种是多行注释，开头结尾各自位于一行：

```cpp
/**
 * struct 张三 - 简短描述.
 * @张三：成员张三.
 */
struct 张三 {
      int 张三;
      /**
       * @李四：成员李四.
       */
      int 李四;
      /**
       * @王五：成员王五.
       *
       * 此处，成员描述可以为好几段.
       */
      int 王五;
      union {
              /** @儿子：单行描述。*/
              int 儿子;
      };
      /** @赵六：描述@张三里面的结构体@赵六 */
      struct {
              /**
               * @赵六。女儿：描述@张三。赵六里面的@女儿
               */
              int 女儿;
      } 赵六;
};
```

注释数据也是很重要的，不管是基本类型还是衍生类型。为了方便实现这一点，每一行 应只声明一个数据 (不要使用逗号来一次声明多个数据)。这样你就有空间来为每个数据 写一段小注释来解释它们的用途了。

## 5.3 文件注释

像下面就可以了，可以不用写 Copyright，反正也没有开源，也没有其他机构使用

```cpp
/*
 * Kernel CAPI 2.0 Module
 *
 * Copyright 1999 by Carsten Paeth <calle@calle.de>
 * Copyright 2002 by Kai Germaschewski <kai@germaschewski.name>
 *
 * This software may be used and distributed according to the terms
 * of the GNU General Public License, incorporated herein by reference.
 *
 */
```

# 6 函数

本章参考
**[Linux 内核 c 代码规范](https://www.kernel.org/doc/html/latest/translations/zh_CN/process/coding-style.html)** 第六章 函数、第七章 集中的函数退出途径、
**[华为 c 代码规范归档版](https://gitee.com/openharmony/docs/blob/master/zh-cn/contribute/OpenHarmony-c-coding-style-guide.md)** 第五章 函数

函数应该简短而漂亮，并且只完成一件事情，要把它做好。
一个函数的最大长度是和该函数的复杂度和缩进级数成反比的。所以，如果你有一个理论上很简单的只有一个很长 (但是简单) 的 case 语句的函数，而且你需要在每个 case 里做很多很小的事情，这样的函数尽管很长，但也是可以的。

不过，如果你有一个复杂的函数，而且你怀疑一个天分不是很高的高中一年级学生可能 甚至搞不清楚这个函数的目的，你应该严格遵守下文函数长度限制。使用辅助函数，并为之取个具描述性的名字 (如果你觉得它们的性能很重要的话，可以让编译器内联它们，这样的效果往往会比你写一个复杂函数的效果要好。)

函数的另外一个衡量标准是本地变量的数量。此数量不应超过 5－10 个，否则你的函数就有问题了。重新考虑一下你的函数，把它分拆成更小的函数。人的大脑一般可以轻松 的同时跟踪 7 个不同的事物，如果再增多的话，就会糊涂了。即便你聪颖过人，你也可 能会记不清你 2 个星期前做过的事情。

在源文件里，使用空行隔开不同的函数。如果该函数需要被导出，它的 EXPORT 宏 应该紧贴在它的结束大括号之下。比如：

```cpp
int system_is_up(void)
{
    return system_state == SYSTEM_RUNNING;
}
EXPORT_SYMBOL(system_is_up);
```

## 6.1 函数最好不超过 50 行

过长的函数往往意味着函数功能不单一，过于复杂，或过分呈现细节，未进行进一步抽象。

例外：
考虑代码的聚合性与功能的全面性，某些函数可能会超过 50 行，但前提是不影响代码的可读性与简洁。
这些例外的函数应该是极少的，例如特定算法处理。

即使一个长函数现在工作的非常好，一旦有人对其修改，有可能出现新的问题，甚至导致难以发现的 bug。
建议将其拆分为更加简短并易于管理的若干函数，以便于他人阅读和修改代码。

## 6.2 函数的代码嵌套最好不超过 4 层

函数的代码块嵌套深度指的是函数中的代码控制块（例如：if、for、while、switch 等）之间互相包含的深度。
**_每级嵌套都会增加阅读代码时的脑力消耗_**，因为需要在脑子里维护一个“栈”（比如，进入条件语句、进入循环等等）。
应该做进一步的功能分解，从而避免使代码的阅读者一次记住太多的上下文。

使用卫语句可以有效的减少 if 相关的嵌套层次。例：
原代码嵌套层数是 3：

```cpp
int foo(...)
{
    if (received) {
        type = get_msg_type(msg);
        if (type != UNKNOWN) {
            return deal_msg(...);
        }
    }
    return -1;
}
```

使用卫语句重构，嵌套层数变成 2：

```cpp
int foo(...)
{
    if (!received) {    // good: 使用'卫语句'
        return -1;
    }

    type = get_msg_type(msg);
    if (type == UNKNOWN) {
        return -1;
    }

    return deal_msg(..);
}
```

例外：
考虑代码的聚合性与功能的全面性，某些函数嵌套可能会超过 4 层，但前提是不影响代码的可读性与简洁。
这些例外的函数应该是极少的。

## 6.3 对函数的错误返回码要全面处理

一个函数（标准库中的函数/第三方库函数/用户定义的函数）能够提供一些指示错误发生的方法。这可以通过使用错误标记、特殊的返回数据或者其他手段，不管什么时候函数提供了这样的机制，调用程序应该在函数返回时立刻检查错误指示。

示例：

```cpp
char file_head[128];
read_file_head(file_name, file_head, sizeof(file_head)); // Bad: 未检查返回值

deal_with_file_head(file_head, sizeof(file_head));       // file_head 可能无效
```

正确写法：

```cpp
char file_head[128];
ret = read_file_head(file_name, file_head, sizeof(file_head));
if (ret != OK) {                // Good: 确保 fileHead 被有效写入
    return ERROR;
}

deal_with_file_head(fileHead, sizeof(fileHead));       // 处理文件头
```

注意，当函数返回值被大量的显式 (void) 忽略掉时，应当考虑函数返回值的设计是否合理。
如果所有调用者都不关注函数返回值时，请将函数设计成 void 型。

## 6.4 函数参数

### 6.4.1 设计函数时，优先使用返回值而不是输出参数

使用返回值而不是输出参数，可以提高可读性，并且通常提供相同或更好的性能。

函数名为 get_xxx、find_xxx、is_xxx、on_xxx 或直接名词作函数名的函数，直接返回对应对象，可读性更好。

例外：

- 1、多值返回时，可以设计为输出参数返回。
- 2、涉及内存分配时，可以设计为输出参数返回。调用者将申请的内存做为入参传入，而函数内由不再分配内存。

### 6.4.2 设计函数的参数时，统一按输入、输出、出入的顺序定义参数

函数参数的定义统一按输入参数、输出参数、出入参的顺序进行定义。

### 6.4.3 设计函数的资源时，涉及内存、锁、队列等资源分配的，需要同时提供释放函数

本着资源从那儿申请，向那儿释放的原则。如果函数申请了内存、锁、队列等资源，则模块需要同时提供资源的函数。

### 6.4.4 使用强类型参数，避免使用 void\*

尽管不同的语言对待强类型和弱类型有自己的观点，但是一般认为 c/c++ 是强类型语言，既然我们使用的语言是强类型的，就应该保持这样的风格。
好处是尽量让编译器在编译阶段就检查出类型不匹配的问题。

使用强类型便于编译器帮我们发现错误，如下代码中注意函数 foo_list_add_node 的使用：

```cpp
struct foo_node {
    struct list link;
    int foo;
};

struct bar_node {
    struct list link;
    int bar;
}

void foo_list_add_node(void *node) // Bad: 这里用 void * 类型传递参数
{
    foo_node *foo = (foo_node *)node;
    list_append(&g_fooList, &foo->link);
}

void make_the_list(...)
{
    foo_node *foo;
    bar_node *bar;
    ...

    foo_list_add_node(bar);        // Wrong: 这里本意是想传递参数 foo，但错传了 bar，却没有报错
}
```

上述问题有可能很隐晦，不易轻易暴露，从而破坏性更大。
如果明确 foo_list_add_node 的参数类型，而不是 void \*，则在编译阶段就能发现上述问题。

```cpp
void foo_list_add_node(foo_node *foo)
{
    list_append(&g_fooList, &foo->link);
}
```

例外：某些通用泛型接口，需要传入不同类型指针的，可以用 void \* 入参。

### 6.4.5 模块内部函数参数的合法性检查，由调用者负责

对于模块外部传入的参数，必须进行合法性检查，保护程序免遭非法输入数据的破坏。
模块内部函数调用，缺省由调用者负责保证参数的合法性，如果都由被调用者来检查参数合法性，可能会出现同一个参数，被检查多次，产生冗余代码，很不简洁。

由调用者保证入参的合法性，这种契约式编程能让代码逻辑更简洁，可读性更好。
示例：

```cpp
int some_proc(...)
{
    int data;

    bool data_ok = get_data(&data); // 获取数据
    if (!data_ok) { // 检查上一步结果，其实也就保证了数据合法
        return -1;
    }

    deal_with_data(data); // 调用数据处理函数
    ...
}

void deal_with_data(int data)
{
    if (data < MIN || data > MAX) {     // Bad: 调用者已经保证了数据合法性
        return;
    }

    ...
}
```

### 6.4.6 函数的指针参数如果不是用于修改所指向的对象就应该声明为指向 const 的指针

const 指针参数，将限制函数通过该指针修改所指向对象，使代码更牢固、安全。

示例：C99 标准 7.21.4.4 中 strncmp 的例子，不变参数声明为 const。

```cpp
int strncmp(const char *s1, const char *s2, size_t n); // Good：不变参数声明为 const
```

注意：指针参数要不要加 const 取决于函数实体内有没有发生“修改对象”的动作。

### 6.4.7 函数的参数个数不超过 5 个

函数的参数过多，会使得该函数易于受外部（其他部分的代码）变化的影响，从而影响维护工作。函数的参数过多同时也会增大测试的工作量。

函数的参数个数不要超过 5 个，如果超过可以考虑：

- 1.看能否拆分函数
- 2.看能否将相关参数合在一起，定义结构体

## 6.5 内联函数

内联函数是 C99 引入的一种函数优化手段。函数内联能消除函数调用的开销；并得益于内联实现跟调用点代码的合并，编译器有更大的视角，从而完成更多的代码优化。内联函数跟函数式宏比较类似。

### 6.5.1 内联函数不超过 10 行（非空非注释）

将函数定义成内联一般希望提升性能，但是实际并不一定能提升性能。如果函数体短小，则函数内联可以有效的缩减目标代码的大小，并提升函数执行效率。
反之，函数体比较大，内联展开会导致目标代码的膨胀，特别是当调用点很多时，膨胀得更厉害，反而会降低执行效率。
内联函数规模建议控制在 10 行以内。

不要为了提高性能而滥用内联函数。不要过早优化。一般情况，当有实际测试数据证明内联性能更高时，再将函数定义为内联。对于类似 setter/getter 短小而且调用频繁的函数，可以定义为内联。

### 6.5.2 被多个源文件调用的内联函数要放在头文件中定义

内联函数是在编译时内联展开，因此要求内联函数的定义必须在调用此函数的每个源文件内可见。
如下所示代码，inline.h 只有 some_inline_func 函数的声明而没有定义。other.c 包含 inline.h，调用 some_inline_func 时无法内联。

```cpp
inline.h

inline int some_inline_func(void);
inline.c

inline int some_inline_func(void)
{
    // 实现代码
}
other.c

#include "inline.h"
int other_func(void)
{
    int ret = some_inline_func();
}
```

由于这个限制，多个源文件如果要调用同一个内联函数，需要将内联函数的定义放在头文件中。
gnu89 在内联函数实现上跟 C99 标准有差异，兼容做法是将函数声明为 static inline。

# 7 宏

本章参考
**[Linux 内核 c 代码规范](https://www.kernel.org/doc/html/latest/translations/zh_CN/process/coding-style.html)** 第十二章 宏，枚举和 RTL
**[华为 c 代码规范归档版](https://gitee.com/openharmony/docs/blob/master/zh-cn/contribute/OpenHarmony-c-coding-style-guide.md)** 第六章 宏

## 7.1 用于定义常量的宏的名字及枚举里的标签需要大写

```cpp
#define CONSTANT 0x12345
```

在定义几个相关的常量时，最好用枚举。

宏的名字请用大写字母，不过形如函数的宏的名字可以用小写字母。

通常如果能写成内联函数就不要写成像函数的宏。

含有多个语句的宏应该被包含在一个 do-while 代码块里：

```cpp
#define macrofun(a, b, c)                       \
        do {                                    \
                if (a == 5)                     \
                        do_this(b, c);          \
        } while (0)
```

## 7.2 使用宏需要注意的事

### 7.2.1 影响控制流程的宏

```cpp
#define FOO(x)                                  \
        do {                                    \
                if (blah(x) < 0)                \
                        return -EBUGGERED;      \
        } while (0)
```

非常 不好。它看起来像一个函数，不过却能导致 调用 它的函数退出；不要打 乱读者大脑里的语法分析器。

### 7.2.2 定义宏时，宏参数要使用完备的括号

```cpp
#define SUM(a, b) ((a) + (b))   // Good.
```

但是要避免滥用括号。如下所示，单独的数字或标识符加括号毫无意义。

```cpp
#define SOME_CONST      100     // Good: 单独的数字无需括号
#define ANOTHER_CONST   (-1)    // Good: 负数需要使用括号

#define THE_CONST SOME_CONST    // Good: 单独的标识符无需括号
```

### 7.2.3 不允许把带副作用的表达式作为参数传递给函数式宏

由于宏只是文本替换，对于内部多次使用同一个宏参数的函数式宏，将带副作用的表达式作为宏参数传入会导致非预期的结果。
如下所示，宏 SQUARE 本身没有问题，但是使用时将带副作用的 a++ 传入导致 a 的值在 SQUARE 执行后跟预期不符：

```cpp
#define SQUARE(a) ((a) * (a))

int a = 5;
int b;
b = SQUARE(a++);    // Bad: 实际 a 自增加了 2 次
SQUARE(a++)展开后为((a++) * (a++))，变量a自增了两次，其值为7，而不是预期的6。
```

正确的写法如下所示：

```cpp
b = SQUARE(a);
a++; // 结果：a = 6，只自增了一次。
```

此外，如果参数包含函数调用，宏展开后，函数可能会被重复调用。
如果函数执行结果相同，则存在浪费；如果函数多次调用结果不一样，执行结果可能不符合预期。

### 7.2.4 函数式宏不超过 10 行 (非空非注释)

函数式宏本身的一大问题是比函数更难以调试和定位，特别是宏过长，调试和定位的难度更大。
而且宏扩展会导致目标代码的膨胀。建议函数式宏不要超过 10 行。

# 8 变量

在 C 语言编码中，除了函数，最重要的就是变量。
变量在使用时，应始终遵循“职责单一”原则。
按作用域区分，变量可分为全局变量和局部变量。

## 8.1 全局变量

### 8.1.1 尽量不用或少用全局变量

在程序设计中，全局变量是在所有作用域都可访问的变量。通常，使用不必要的全局变量被认为是坏习惯。

使用全局变量的缺点：

- 破坏函数的独立性和可移植性，使函数对全局变量产生依赖，存在耦合；
- 降低函数的代码可读性和可维护性。当多个函数读写全局变量时，某一时刻其取值可能不是确定的，对于代码的阅读和维护不利；
- 在并发编程环境中，使用全局变量会破坏函数的可重入性，需要增加额外的同步保护处理才能确保数据安全。
  如不可避免，对全局变量的读写应集中封装。

### 8.1.2 模块间，禁止使用全局变量作接口

全局变量是模块内部的具体实现，不推荐但允许跨文件使用，但禁止作为模块接口暴露出去。
对全局变量的使用应该尽量集中，如果本模块的数据需要对外部模块开放，应提供对应函数接口。

## 8.2 局部变量

### 8.2.1 严禁使用未经初始化的变量

这里的变量，指的是局部动态变量，并且还包括内存堆上申请的内存块。
因为他们的初始值都是不可预料的，所以禁止未经有效初始化就直接读取其值。

```cpp
void Foo(...)
{
    int data;
    bar(data);  // Bad: 未初始化就使用
    ...
}
```

如果有不同分支，要确保所有分支都得到初始化后才能使用：

```cpp
void Foo(...)
{
    int data;
    if (...) {
        data = 100;
    }
    bar(data);  // Bad: 部分分支该值未初始化
    ...
}
```

未经初始化就使用，一般静态检查工具是可以检查出来的。
如 PCLint 工具，针对上述两个例子分别会报错：

Warning 530: Symbol 'data' (line ...) not initialized Warning 644: Variable 'data' (line ...) may not have been initialized

### 8.2.2 禁止无效、冗余的变量初始化

如果没有确定的初始值，而仍然进行初始化，不仅不简洁，反而不安全，可能会引入更难发现的问题。

常见的冗余初始化：

```cpp
int cnt = 0;    // Bad: 冗余初始化，将会被后面直接覆盖
...
cnt = get_xxx_cnt();
...
```

对于后续有条件赋值的变量，可以在定义时初始化成默认值

```cpp
char *buf = NULL;   // Good: 这里用 NULL 代表默认值
if (condition) {
    buf = malloc(MEM_SIZE);
}
...
if (buf != NULL) {  // 判断是否申请过内存
    free(buf);
}
```

针对大数组的冗余清零，更是会影响到性能。

```cpp
char buf[VERY_BIG_SIZE] = {0};
memset(buf, 0, sizeof(buf));    // Bad: 冗余清零
```

无效初始化，隐藏更大问题的反例：

```cpp
void foo(...)
{
    int data = 0;   // Bad: 习惯性的进行初始化

    use_data(data);          // 使用数据，本应该写在获取数据后面
    data = get_data(...);    // 获取数据
    ...
}
```

上例代码，如果没有赋 0 初始化，静态检查工具可以帮助发现“未经初始化就直接使用”的问题。
但因为无效初始化，“使用数据”与“获取数据”写颠倒的缺陷，不能被轻易发现。

因此，应该写简洁的代码，对变量或内存块进行正确、必要的初始化。

C99 不再限制局部变量定义必须在语句之前，可以按需定义，即在靠近变量使用的地方定义变量。
这种简洁的做法，不仅将变量作用域限制更小，而且更方便阅读和维护，还能解决定义变量时不知该怎么初始化的问题。
如果编译环境支持，建议按需定义。

例外：
遵从“安全规范”要求，指针变量、表示资源描述符的变量、BOOL 变量不作要求。

### 8.2.3 不允许使用魔鬼数字

所谓魔鬼数字即看不懂、难以理解的数字。
魔鬼数字并非一个非黑即白的概念，看不懂也有程度，需要结合代码上下文和业务相关知识来判断

例如数字 12，在不同的上下文中情况是不一样的：
type = 12; 就看不懂，但 month = year \* 12; 就能看懂。
数字 0 有时候也是魔鬼数字，比如 status = 0; 并不能表达是什么状态。

解决途径：
对于单点使用的数字，可以增加注释说明
对于多处使用的数字，必须定义宏或 const 变量，并通过符号命名自注释。

禁止出现下列情况：
没有通过符号来解释数字含义，如 #define ZERO 0
符号命名限制了其取值，如 #define XX_TIMER_INTERVAL_300MS 300
