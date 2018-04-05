import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import csv
from add_layer import add_layer
from sklearn.cross_validation import train_test_split


ShoulderPitch_DeltaTheta = []
ShoulderPitch_Alpha = []

with open('RArm_Shoulder_Pitch.txt', 'r') as csvfile:

    plots = csv.reader(csvfile, delimiter=',')

    for row in plots:
        ShoulderPitch_DeltaTheta.append(row[0])
        ShoulderPitch_Alpha.append(row[1])

temp = np.shape(ShoulderPitch_DeltaTheta)
ShoulderPitch_DeltaTheta = np.reshape(ShoulderPitch_DeltaTheta, (temp[0], 1))

temp1 = np.shape(ShoulderPitch_Alpha)
ShoulderPitch_Alpha = np.reshape(ShoulderPitch_Alpha, (temp1[0], 1))

ShoulderPitch_DeltaTheta_train, ShoulderPitch_DeltaTheta_test, ShoulderPitch_Alpha_train, ShoulderPitch_Alpha_test = train_test_split(
    ShoulderPitch_DeltaTheta, ShoulderPitch_Alpha, test_size=0.2)

# define placeholder for inputs to network
with tf.name_scope('inputs'):
    xs_Shoulder_pitch = tf.placeholder(tf.float32, shape=[None, 1], name="x_input")
    ys_Shoulder_Pitch = tf.placeholder(tf.float32, shape=[None, 1], name="y_input")

# add hidden layer
layer_1 = add_layer(xs_Shoulder_pitch, 1, 25, n_layer=1, activation_function=tf.nn.sigmoid)
# add output layer
prediction_Shoulder_Pitch = add_layer(layer_1, 25, 1, n_layer=2, activation_function=None)

tf.add_to_collection("Shoulder_Pitch_input", xs_Shoulder_pitch)
tf.add_to_collection("Shoulder_Pitch_prediction", prediction_Shoulder_Pitch)

# the error between prediction and real data
with tf.name_scope('loss'):
    loss = tf.reduce_mean(tf.reduce_sum(tf.square(ys_Shoulder_Pitch - prediction_Shoulder_Pitch),
                                        reduction_indices=[1]))
    tf.summary.scalar('loss', loss)

with tf.name_scope('train'):
    train_step = tf.train.AdamOptimizer(0.001).minimize(loss)

saver = tf.train.Saver()

sess = tf.Session()

merged = tf.summary.merge_all()
writer = tf.summary.FileWriter("TensorBoard", sess.graph)

# tf.initialize_all_variables()
init = tf.global_variables_initializer()
sess.run(init)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.scatter(ShoulderPitch_DeltaTheta, ShoulderPitch_Alpha)
plt.ion()
plt.grid()
plt.show()
lines = []

for epoch in range(30001):
    # training
    sess.run(train_step, feed_dict={xs_Shoulder_pitch: ShoulderPitch_DeltaTheta_train,
                                    ys_Shoulder_Pitch: ShoulderPitch_Alpha_train})
    if epoch % 500 == 0:

        result = sess.run(merged,
                          feed_dict={xs_Shoulder_pitch: ShoulderPitch_DeltaTheta_train,
                                     ys_Shoulder_Pitch: ShoulderPitch_Alpha_train})
        writer.add_summary(result, epoch)

        # to see the step improvement
        print("Epoch:", epoch, "Loss_train:", sess.run(loss, feed_dict={xs_Shoulder_pitch: ShoulderPitch_DeltaTheta_train,
                                                                        ys_Shoulder_Pitch: ShoulderPitch_Alpha_train}),
              'Loss_test:', sess.run(loss, feed_dict={xs_Shoulder_pitch: ShoulderPitch_DeltaTheta_test,
                                                      ys_Shoulder_Pitch: ShoulderPitch_Alpha_test}))

        try:
            ax.lines.remove(lines[0])
        except Exception:
            pass
        prediction_value_train = sess.run(prediction_Shoulder_Pitch,
                                          feed_dict={xs_Shoulder_pitch: ShoulderPitch_DeltaTheta_train})
        # plot the prediction
        # Print "prediction_value:", prediction_value
        lines = ax.plot(ShoulderPitch_DeltaTheta_train, prediction_value_train, 'r.', lw=0.5)
        plt.pause(0.5)

save_path = saver.save(sess, "NNweights/R_Shoulder_Pitch/R_Shoulder_Pitch.ckpt")
