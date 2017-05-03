choice_answer_mapping = {

	# Predefined questions:

	"What city are we in?": "Magdeburg",
	"What is the name of your team?": "Tech United",
	"Name the big hairy creature in Star Wars.": "Chewbacca",
	"Who wrote the three laws of robotics?": "Isaac Asimov",
	"From what series do you know Rosie the robot?": "The Jetsons",
	"From what series do you know the baby Bam Bam": "The Flintstones",
	"Who is the main charcater of The Matrix": "Neo",
	"Name the two RoboCup@Home standart platforms.": "Pepper and HSR",
	"Where do you store your memories": "In my SSD",
	"Where is your team located?": "In Eindhoven The Netherlands",

	# Crowd questions:

	# "How many $people are in the crowd?"
	# "How many people in the crowd are $posppl | {gesture}?"
	# "Tell me the number of $people in the crowd"
	# "The $posprs person was $gprsng?"
	# "Tell me if the ($posprs | {gesture}) person was a $gprsn?"
	# "Tell me how many people were wearing $color"

# $people = $appl | $gppl
# $appl   = children | adults | elders
# $gppl   = males | females | men | women | boys | girls
# $posppl = $posprs
# $posppl = standing or sitting
# $posppl = standing or lying
# $posppl = sitting or lying
# $posprs = standing | sitting | lying
# $gprsn  = male | female | man | woman | boy | girl
# $gprsng = male or female | man or woman | boy or girl
# $color  = red | blue | white | black | green | yellow

	"How many males are there in the crowd?": "Crowd_males",
	"How many females are there in the crowd?": "Crowd_females",
	"What is the size of the crowd?": "Crowd_size",
	"How many children are there in the crowd?": "Crowd_children"

	# Object questions:

	# "Where can I find a {object}?"
	# "How many {category} there are?"
	# "Whats the colour of the {kobject}?"
	# "How many ({category} | objects) are in the {placement}?"
	# "What objects are stored in the {placement}?"
	# "Where can I find a ({object} | {category})?"
	# "To which category belong the {object}?"
	# "Do the {object 1} and {object 2} belong to the same category?"
	# "Which is the $adja ({category} | object)?"
	# "Between the {object 1} and {object 2}, which one is $adjr?"

# $adja = heaviest | smallest | biggest | lightest
# $adjr = heavier | smaller | bigger | lighter

	# Arena questions:

	# "Where is the {placement}?"
	# "Where is the {beacon}?"
	# "In which room is the {placement}?"
	# "In which room is the {beacon}?"
	# "How many doors has the {room}?"
	# "How many ({placement} | {beacon}) are in the {room}?"

}
