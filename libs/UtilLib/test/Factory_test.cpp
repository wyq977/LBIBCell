/*
 *
 * Copyright (c) 2011 David-Matthias Sichau
 * Copyright (c) 2010 Marc Kirchner
 *
 * This file is part of libpipe.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <UtilLib/include/Factory.hpp>
#include <UtilLib/include/Singleton.hpp>

#include <boost/test/minimal.hpp>
#include <iostream>
#include <string>
#include <exception>

using namespace boost::unit_test;
using namespace UtilLib;

class AbstractType;

typedef Singleton<Factory<AbstractType, std::string> > AbstractFactory;

class AbstractType {
 public:
    virtual ~AbstractType() = 0;

    virtual std::string getName() = 0;
};

AbstractType::~AbstractType() {}

class ConcreteType : public AbstractType {
    // virtual ~ConcreteType(){}

    static AbstractType* create() {
        return new ConcreteType;
    }

    virtual std::string getName() {
        return "test";
    }

 private:
    static const bool registered_;

    static bool registerLoader() {
        std::string ids = "test";
        return AbstractFactory::instance().registerType(ids,
                ConcreteType::create);
    }
};

const bool ConcreteType::registered_ = registerLoader();


void test() {
    std::string ids = "test";
    AbstractType* t = AbstractFactory::instance().createObject(ids);
    BOOST_CHECK(t->getName() == "test");
    delete t;
}

void testUnregister() {
    std::string ids = "test";
    bool thrown = false;
    AbstractType* t = AbstractFactory::instance().createObject(ids);
    AbstractFactory::instance().unregisterType(ids);
    try {
        t = AbstractFactory::instance().createObject(ids);
    } catch (std::bad_typeid& e) {
        thrown = false;
    }

    BOOST_CHECK(thrown == false);

    delete t;
}

void testWrongType() {
    std::string ids = "blubb";
    bool thrown = false;
    try {
        AbstractType* t = AbstractFactory::instance().createObject(ids);
        delete t;
    } catch (std::exception& e) {
        thrown = true;
    }

    BOOST_CHECK(thrown == true);
}

int test_main(
        int argc,
        char* argv[]) { // note the name!
    // we use only two processors for this testing
    test();
    testUnregister();
    testWrongType();


    return 0;
    //    // six ways to detect and report the same error:
    //    BOOST_CHECK( add( 2,2 ) == 4 );        // #1 continues on error
    //    BOOST_CHECK( add( 2,2 ) == 4 );      // #2 throws on error
    //    if( add( 2,2 ) != 4 )
    //        BOOST_ERROR( "Ouch..." );          // #3 continues on error
    //    if( add( 2,2 ) != 4 )
    //        BOOST_FAIL( "Ouch..." );           // #4 throws on error
    //    if( add( 2,2 ) != 4 ) throw "Oops..."; // #5 throws on error
    //
    //    return add( 2, 2 ) == 4 ? 0 : 1;       // #6 returns error code
}
