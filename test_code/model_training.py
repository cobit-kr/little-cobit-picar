__author__ = 'zhengwang'

from model import load_data, NeuralNetwork

input_size = 148 * 400
data_path = "training_data/*.npz"

'''
model.py 참고 
load_data(): npz에 저장된 이미지/방향 두 데이터 그룹을 로드하고, 7:3의 비율로 
train test로 나눈다. train 그룹 데이터는 학습에 사용되고, test 그룹 데이터는 테스트용도로 사용한다.  
'''
X_train, X_valid, y_train, y_valid = load_data(input_size, data_path)

# train a neural network
'''
신경망을 만든다. 입력은 이미지 데이터 사이즈: image_size
출력은 방향 데이터 2

'''
layer_sizes = [input_size, 32, 2]
nn = NeuralNetwork()
nn.create(layer_sizes)
'''
먼저 학습 데이터 그룹의 이미지 데이터(X_train)과 방향 데이터(y_train)으로 학습한다. 
nn.train()은 다음과 같이 동작한다. 
1. 임의의 weight와 bias를 신경망에 설정한다. 
2. 입력 데이터를 신경망에 넣고 계산해서 출력을 얻는다. 
3. 이 출력이 방향 데이터와 유사한지 확인한다. 
4. 출력값과 방향 데이터가 다르면 weight와 bias를 변경해서 다시 계산한다. 
5. 최적의 상태가 나올때까지 반복한다(back propagation).
'''
nn.train(X_train, y_train)

# evaluate on train data
'''
이미 학습한 X_train, y_train 데이터로 predict() 해본다. 
원래는 X_valid, y_valid로 PREDICT()하지민 X_train, y_train으로 
predict() 해본 것이다. 실제로 100% 값 나오지 않는다. 
'''
train_accuracy = nn.evaluate(X_train, y_train)
print("Train accuracy: ", "{0:.2f}%".format(train_accuracy * 100))

# evaluate on validation data
'''
X_vaild와 y_valid로 predict()를 한다. 
1. X_valid를 앞에서 train에서 정해진 weight bias값으로 forward propagation을 한다. 
2. Forward propagation 출력값에 따라서 자율주행을 하면 된다. 
3. 정확도 계산을 위해서 Forward propagation에서 출력된 값과 y_train과 비교한다. 
'''
validation_accuracy = nn.evaluate(X_valid, y_valid)
print("Validation accuracy: ", "{0:.2f}%".format(validation_accuracy * 100))

# save model
model_path = "saved_model/nn_model.xml"
nn.save_model(model_path)
