from transitions import Machine

class Matter(object):
    pass
model = Matter()


states=['solid', 'liquid', 'gas', 'plasma']


# The trigger argument defines the name of the new triggering method
transitions = [
    {'trigger': 'none', 'source': 'solid', 'dest': 'solid' },
    {'trigger': 'melt', 'source': 'solid', 'dest': 'liquid' },
    {'trigger': 'evaporate', 'source': 'liquid', 'dest': 'gas'},
    {'trigger': 'sublimate', 'source': 'solid', 'dest': 'gas'},
    {'trigger': 'ionize', 'source': 'gas', 'dest': 'plasma'}]


machine = Machine(model=model, states=states, transitions=transitions, initial='solid')


# Test 
state = model.state    # solid
print(state)

model.none()

state = model.state   # solid
print(state)

model.melt()

state = model.state   # liquid
print(state)
