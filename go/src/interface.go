//
// https://github.com/luciotato/golang-notes/blob/master/OOP.md
//
package main

import (
	"fmt"
  "reflect"
)

/////////////////////////////////
type Organism interface {
  IsLiving() bool
  ToString() string
  // Uncomment to get compilation failure
  //Location() float32
}

/////////////////////////////////
// class Animal - parent interface
type Animal interface {
  Organism
	Speak() string
  // Uncomment to get compilation failure
  //Weight() int32
}

/////////////////////////////////
// class Dog
type Dog struct {
  sound string
  age int32
}

// implement parent Organism methods
func (d Dog) IsLiving() bool {
	return true
}

// implement parent Animal methods
func (d Dog) Speak() string {
	return d.sound
}

func (d Dog) ToString() string {
	return fmt.Sprintf("%v %v %v", d.sound, d.age, d.IsLiving())
}

// additional methods
func (d Dog) Age() int32 {
	return d.age
}

// Default constructor
func NewDog() Dog {
  obj := Dog{}
  obj.sound = "Woof!"
  obj.age = 100
  return obj
}

/////////////////////////////////
// class Cat
type Cat struct {
  sound string
}

// implement parent Organism methods
func (c Cat) IsLiving() bool {
	return false
}

// implement parent Animal methods
func (c Cat) Speak() string {
	return c.sound
}

func (d Cat) ToString() string {
	return fmt.Sprintf("%v", d.sound)
}

// Default constructor
func NewCat() Cat {
  obj := Cat{}
  obj.sound = "Meow!"
  return obj
}

/////////////////////////////////
// class Llama
type Llama struct {
  sound string
  height int32
}

// implement parent Organism methods
func (l Llama) IsLiving() bool {
	return true
}

// implement parent Animal methods
func (l Llama) Speak() string {
	return l.sound
}

func (d Llama) ToString() string {
	return fmt.Sprintf("%v %v", d.sound, d.height)
}

// additional methods
func (d Llama) Height() int32 {
	return d.height
}

// Default constructor
func NewLlama() Llama {
  obj := Llama{}
  obj.sound = "LaLLamaQueLLama!"
  obj.height = 333
  return obj
}

/////////////////////////////////
// main
func main() {
  // Create objects that are the implementation classes
	organisms := []Organism{ NewDog(), NewCat(), NewLlama() }

  // Iterate over the object list as the parent interface class
	for _, organism := range organisms {

    // Just use the parent interface methods
		fmt.Print(fmt.Sprintf("%v  %v -- ", reflect.TypeOf(organism).String(), organism.IsLiving())) // method dispatch via jmp-table
		fmt.Print(organism.ToString() + "  --  ") // method dispatch via jmp-table

		// Cast it to an instance of the implementation interface
		animal := organism.(Animal)
		fmt.Println(reflect.TypeOf(animal).String() + "  " + animal.Speak()) // method dispatch via jmp-table
	}
}
