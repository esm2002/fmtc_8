import os
import numpy as np
import cv2
from sklearn.model_selection import train_test_split, KFold
from tensorflow.keras.utils import to_categorical
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout, BatchNormalization
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau#, ModelCheckpoint

# 데이터 로드 함수 (이전과 동일)
def load_data(image_dir):
    images = []
    steer_values = []
    for filename in os.listdir(image_dir):
        if filename.endswith('.jpg'):
            img_path = os.path.join(image_dir, filename)
            img = cv2.imread(img_path)
            steer_path = filename.replace('.jpg', '.txt')
            with open(os.path.join(image_dir, steer_path), 'r') as f:
                steer_value = int(f.read())
            
            images.append(img)
            steer_values.append(steer_value)
    
    return np.array(images), np.array(steer_values)

# 데이터 불러오기
image_dir = '/path/to/save/images'  # 여기에 수집한 데이터가 저장된 경로를 입력하세요.
X, y = load_data(image_dir)

# 데이터 전처리 (이미지 정규화)
X = X / 255.0

# 조향값을 원-핫 인코딩
y = to_categorical(y, num_classes=3)

# 데이터 증강 설정
datagen = ImageDataGenerator(
    rotation_range=10,
    width_shift_range=0.1,
    height_shift_range=0.1,
    horizontal_flip=True,
    zoom_range=0.1
    #shear_range=0.1,  # 추가: 이미지 왜곡을 통해 더 다양한 학습 데이터를 생성합니다.
    #brightness_range=[0.8, 1.2]  # 추가: 밝기 조정을 통해 모델이 다양한 조명 조건에서 잘 작동하도록 합니다.
)

# 모델 정의 함수
def create_model():
    model = Sequential([
        Conv2D(32, (3, 3), activation='relu', input_shape=(480, 640, 3)),
        BatchNormalization(),
        MaxPooling2D((2, 2)),
        Conv2D(64, (3, 3), activation='relu'),
        BatchNormalization(),
        MaxPooling2D((2, 2)),
        Conv2D(128, (3, 3), activation='relu'),
        BatchNormalization(),
        MaxPooling2D((2, 2)),
        Flatten(),
        Dense(512, activation='relu'),
        Dropout(0.5),
        #Dense(256, activation='relu'),  # 추가: Dense 레이어를 하나 더 추가하여 모델의 복잡도를 높입니다.
        #Dropout(0.5),  # 추가된 Dense 레이어 뒤에도 Dropout을 적용합니다.
        Dense(3, activation='softmax')  # 좌회전, 직진, 우회전
    ])
    
    model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
    return model

    #optimizer = Adam(learning_rate=0.001)
    #model.compile(optimizer=optimizer, loss='categorical_crossentropy', metrics=['accuracy'])
    #return model


# K-fold 교차 검증 설정
k_fold = KFold(n_splits=5, shuffle=True, random_state=42)

# 콜백 설정
early_stopping = EarlyStopping(patience=5, restore_best_weights=True)
reduce_lr = ReduceLROnPlateau(factor=0.2, patience=3, min_lr=0.00001)
#checkpoint = ModelCheckpoint('best_model_fold_{fold_no}.h5', save_best_only=True, monitor='val_loss', mode='min')  # 추가: 각 폴드에서 가장 성능이 좋은 모델을 저장합니다.


# K-fold 교차 검증 수행
fold_no = 1
accuracies = []

for train_index, val_index in k_fold.split(X):
    print(f'FOLD {fold_no}')
    X_train, X_val = X[train_index], X[val_index]
    y_train, y_val = y[train_index], y[val_index]
    
    model = create_model()
    
    history = model.fit(
        datagen.flow(X_train, y_train, batch_size=32),
        epochs=50,
        validation_data=(X_val, y_val),
        callbacks=[early_stopping, reduce_lr] #callbacks=[early_stopping, reduce_lr, checkpoint]  # ModelCheckpoint 콜백을 추가했습니다.
    )
    
    # 모델 평가
    loss, accuracy = model.evaluate(X_val, y_val)
    print(f'Validation Loss: {loss}')
    print(f'Validation Accuracy: {accuracy}')
    accuracies.append(accuracy)
    
    fold_no += 1

# 평균 정확도 출력
print(f'Average accuracy: {np.mean(accuracies)}')

# 최종 모델 훈련 (전체 데이터셋 사용)
final_model = create_model()
final_history = final_model.fit(
    datagen.flow(X, y, batch_size=32),
    epochs=50,
    callbacks=[early_stopping, reduce_lr] #callbacks=[early_stopping, reduce_lr, checkpoint]  # ModelCheckpoint 콜백을 추가했습니다.
)

# 최종 모델 저장
final_model.save('new_best_model.h5')