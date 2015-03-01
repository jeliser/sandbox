# Hello World sample application

load 'Greeter.rb'

@you = Greeter.new()
@you.say_hi()
@you.say_bye()

me = Greeter.new("Josh")
me.say_hi
me.say_bye

# Looks like local and instance variables are both valid methods to save instances.

#puts Greeter.instance_methods
puts Greeter.instance_methods(false)

puts me.respond_to?("say_hi")
puts me.respond_to?("to_s")

# Allow name to be accessable
puts me.respond_to?("name")
puts me.respond_to?("name=")
class Greeter
  # Crazy ass runtime modification of a classes' instance variables.
  attr_accessor :name
end
puts me.respond_to?("name")
puts me.respond_to?("name=")

me.say_hi
me.name = "Tucker"
me.say_hi


