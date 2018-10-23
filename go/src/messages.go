//
// https://github.com/luciotato/golang-notes/blob/master/OOP.md
//
package main

import (
  "fmt"
  "reflect"
)

/////////////////////////////////
type MessageInterface interface {
  IsActive() bool
  GetWellName() string
  SetWellName(string)
  ToString() string
}

/////////////////////////////////
// class Record01Interface - parent interface
type Record01Interface interface {
  MessageInterface
  GetDrillSpeed() float32
  SetDrillSpeed(float32)
}

/////////////////////////////////
// class Record01_pb
type Record01_pb struct {
  well_name string
  drill_speed float32
}

// Implement MessageInterface methods
func (d Record01_pb) IsActive() bool {
  return true
}

func (d Record01_pb) GetWellName() string {
  return d.well_name
}

func (d *Record01_pb) SetWellName(val string) {
  d.well_name = val
}

func (d Record01_pb) ToString() string {
  return fmt.Sprintf("%v %v %v", d.GetWellName(), d.GetDrillSpeed(), d.IsActive())
}

// Implement the Record01Interface methods
func (d Record01_pb) GetDrillSpeed() float32 {
  return d.drill_speed
}

func (d *Record01_pb) SetDrillSpeed(val float32) {
  d.drill_speed = val
}

// Default constructor (must be called explicitly)
func NewRecord01_pb() *Record01_pb {
  obj := &Record01_pb{}
  obj.SetWellName("BP/WELL01")
  obj.SetDrillSpeed(100)
  return obj
}

/////////////////////////////////
// class Record02Interface - parent interface
type Record02Interface interface {
  MessageInterface
  GetDrillDepth() int32
  SetDrillDepth(int32)
}

/////////////////////////////////
// class Record02_pb
type Record02_pb struct {
  well_name string
  drill_depth int32
}

// Implement MessageInterface methods
func (d Record02_pb) IsActive() bool {
  return true
}

func (d Record02_pb) GetWellName() string {
  return d.well_name
}

func (d *Record02_pb) SetWellName(val string) {
  d.well_name = val
}

func (d Record02_pb) ToString() string {
  return fmt.Sprintf("%v %v %v", d.GetWellName(), d.GetDrillDepth(), d.IsActive())
}

// Implement the Record01Interface methods
func (d Record02_pb) GetDrillDepth() int32 {
  return d.drill_depth
}

func (d *Record02_pb) SetDrillDepth(val int32) {
  d.drill_depth = val
}

// Default constructor (must be called explicitly)
func NewRecord02_pb() *Record02_pb {
  obj := &Record02_pb{}
  obj.SetWellName("ANA/NORTH444")
  obj.SetDrillDepth(333)
  return obj
}

/////////////////////////////////
// class Record03Interface - parent interface
type Record03Interface interface {
  MessageInterface
  GetUptime() int64
  SetUptime(int64)
}

/////////////////////////////////
// class Record03_pb
type Record03_pb struct {
  well_name string
  uptime int64
}

// Implement MessageInterface methods
func (d Record03_pb) IsActive() bool {
  return true
}

func (d Record03_pb) GetWellName() string {
  return d.well_name
}

func (d *Record03_pb) SetWellName(val string) {
  d.well_name = val
}

func (d Record03_pb) ToString() string {
  return fmt.Sprintf("%v %v %v", d.GetWellName(), d.GetUptime(), d.IsActive())
}

// Implement the Record01Interface methods
func (d Record03_pb) GetUptime() int64 {
  return d.uptime
}

func (d *Record03_pb) SetUptime(val int64) {
  d.uptime = val
}

// Default constructor (must be called explicitly)
func NewRecord03_pb() *Record03_pb {
  obj := &Record03_pb{}
  obj.SetWellName("SHELL/GULF54")
  obj.SetUptime(123456789)
  return obj
}


/////////////////////////////////
// main
func main() {
  // Create objects that are the implementation classes
  messages := []MessageInterface{ NewRecord01_pb(), NewRecord02_pb(), NewRecord03_pb() }

  // Iterate over the object list as the parent interface class
  for _, message := range messages {

    // Just use the parent interface methods
    fmt.Print(fmt.Sprintf("%v  %v -- ", reflect.TypeOf(message).String(), message.IsActive()))
    fmt.Print(message.ToString() + "  --  ")

    // Get the underlying object type
    switch reflect.TypeOf(message) {
      case reflect.TypeOf(&Record01_pb{}):
        record01 := message.(Record01Interface)
        fmt.Println(fmt.Sprintf("%v  %v", reflect.TypeOf(record01).String(), record01.GetDrillSpeed()))
      case reflect.TypeOf(&Record02_pb{}):
        record02 := message.(Record02Interface)
        fmt.Println(fmt.Sprintf("%v  %v", reflect.TypeOf(record02).String(), record02.GetDrillDepth()))
      case reflect.TypeOf(&Record03_pb{}):
        record03 := message.(Record03Interface)
        fmt.Println(fmt.Sprintf("%v  %v", reflect.TypeOf(record03).String(), record03.GetUptime()))
      default:
        fmt.Println(fmt.Sprintf("Unhandled type: %v", reflect.TypeOf(message)))
    }
  }
}
