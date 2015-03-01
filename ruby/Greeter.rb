class Greeter

  #attr_accessor :name

  def initialize(name = "world")
    # @name is an instance variable accessable by the whole class
    @name = name
  end

  def say_hi
    puts "Hello, #{@name.capitalize()}"
  end

  def say_bye
    puts "Goodbye, #{@name.capitalize()}"
  end
end



