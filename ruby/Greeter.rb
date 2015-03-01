class Greeter

  # Makes the names attribute public
  # me = Greeter.new()
  # me.names = "Josh"
  #attr_accessor :names

  def initialize(names = "world")
    # @names is an instance variable accessable by the whole class
    @names = names
  end

  def say_hi
    say("Hello")
  end

  def say_bye
    say("Goodbye")
  end

  private
    def say(greeting)
      if @names.nil?
        puts "Please load a name or names to be greeted."
      elsif @names.respond_to?("each")
        if greeting == "Hello"
          @names.each do |name|
            puts "#{greeting}, #{name.capitalize()}"
          end
        else
          puts "#{greeting}, #{@names.join(", ")}"
        end
      else
        puts "#{greeting}, #{@names.capitalize()}"
      end     
    end
end


# Pretty cool way having a driver test for the class
if __FILE__ == $0
  me = Greeter.new("Josh")
  me.say_hi
  me.say_bye

  # Looks like local and instance variables are both valid methods to save instances.

  #puts Greeter.instance_methods
  puts Greeter.instance_methods(false)

  puts me.respond_to?("say_hi")
  puts me.respond_to?("to_s")

  # Allow name to be accessable
  puts me.respond_to?("names")
  puts me.respond_to?("names=")
  class Greeter
    # Crazy ass runtime modification of a classes' instance variables.
    attr_accessor :names
  end
  puts me.respond_to?("names")
  puts me.respond_to?("names=")

  me.say_hi
  me.names = ["Monk", "Gidget", "Tucker"]
  me.say_hi
  me.say_bye

end
