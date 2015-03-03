from datetime import datetime
choice_answer_mapping = {
	"what time is it":"It is %s"%datetime.now().strftime("%I %M %p"),
	"what is the capital of germany":"berlin",
	"what is the heaviest animal in the world":"The blue whale",
	"who is the president of america":"barack obama",
	"what is your name":"It's me Amigo",
	"who is the ugliest person in the world":"Rein Appeldoorn",
	"what is your motto":"I am a banana!",
	"which football club is the best":"Ajax",
	"who is the best looking person around here":"Simple question, Erik Geerts",
	"which team member is your favorite operator":"that i cannot choose, but if I have to, it would be my dear Erik",
	"which town has been bombed":"Rotterdam and Schijndel",
	"which person is not able to say yes":"Dirk Holtz",
	"what is the capital of brazil":"brasilia",
	"what is the oldest drug used on earth":"alcohol",
	"in which year was robocup founded":"nineteen ninety seven",
	"how many rings has the olympic flag":"five",
	"what is the worlds most popular green vegetable":"lettuce",
	"which insect has the best eyesight":"dragonfly",
	"who lives in a pineapple under the sea":"spongebob squarepants",
	"what is your teams name":"Tech united eindhoven",
	"what is the answer to the ultimate question about life the universe and everything":"forty two",
	"what is the capital of poland":"warsaw",
	"which country grows the most potatoes":"russia",
	"which country grew the first orange":"china",
	"how many countries are in europe":"fifty",
	"which fish can hold objects in its tail":"sea horse"
}

spec = '<question>'
choices = {'question': [k for k,v in choice_answer_mapping.iteritems()]}


