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
        @names.each do |name|
          puts "#{greeting}, #{name.capitalize()}"
        end
      else
        puts "#{greeting}, #{@names.capitalize()}"
      end     
    end
end



