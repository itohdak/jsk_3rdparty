#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import actionlib
import rospy
import speech_recognition as SR
import json
from threading import Lock

from audio_common_msgs.msg import AudioData
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from speech_recognition_msgs.srv import SpeechRecognition
from speech_recognition_msgs.srv import SpeechRecognitionResponse

from dynamic_reconfigure.server import Server
from ros_speech_recognition.cfg import SpeechRecognitionConfig as Config

from std_msgs.msg import Bool

import io
import os
import sys
import subprocess
import wave
import aifc
import math
import audioop
import collections
import json
import base64
import threading
import platform
import stat
import hashlib
import hmac
import time
import uuid

class MyRecognizer(SR.Recognizer): # overloading SR.Recognizer.listen and SR.Recognizer.listen_in_background
    def __init__(self, _loud_rate):
        super(MyRecognizer, self).__init__()
        self.is_speaking = False
        self.is_speaking_loud = False
        self.loud_rate = _loud_rate

    @property
    def energy_threshold_loud(self):
        return self.energy_threshold * self.loud_rate

    def listen(self, source, timeout=None, phrase_time_limit=None, snowboy_configuration=None, publisher=None, publisher_loud=None):
        """
        Records a single phrase from ``source`` (an ``AudioSource`` instance) into an ``AudioData`` instance, which it returns.

        This is done by waiting until the audio has an energy above ``recognizer_instance.energy_threshold`` (the user has started speaking), and then recording until it encounters ``recognizer_instance.pause_threshold`` seconds of non-speaking or there is no more audio input. The ending silence is not included.

        The ``timeout`` parameter is the maximum number of seconds that this will wait for a phrase to start before giving up and throwing an ``speech_recognition.WaitTimeoutError`` exception. If ``timeout`` is ``None``, there will be no wait timeout.

        The ``phrase_time_limit`` parameter is the maximum number of seconds that this will allow a phrase to continue before stopping and returning the part of the phrase processed before the time limit was reached. The resulting audio will be the phrase cut off at the time limit. If ``phrase_timeout`` is ``None``, there will be no phrase time limit.

        The ``snowboy_configuration`` parameter allows integration with `Snowboy <https://snowboy.kitt.ai/>`__, an offline, high-accuracy, power-efficient hotword recognition engine. When used, this function will pause until Snowboy detects a hotword, after which it will unpause. This parameter should either be ``None`` to turn off Snowboy support, or a tuple of the form ``(SNOWBOY_LOCATION, LIST_OF_HOT_WORD_FILES)``, where ``SNOWBOY_LOCATION`` is the path to the Snowboy root directory, and ``LIST_OF_HOT_WORD_FILES`` is a list of paths to Snowboy hotword configuration files (`*.pmdl` or `*.umdl` format).

        This operation will always complete within ``timeout + phrase_timeout`` seconds if both are numbers, either by returning the audio data, or by raising a ``speech_recognition.WaitTimeoutError`` exception.
        """
        assert isinstance(source, SR.AudioSource), "Source must be an audio source"
        assert source.stream is not None, "Audio source must be entered before listening, see documentation for ``AudioSource``; are you using ``source`` outside of a ``with`` statement?"
        assert self.pause_threshold >= self.non_speaking_duration >= 0
        if snowboy_configuration is not None:
            assert os.path.isfile(os.path.join(snowboy_configuration[0], "snowboydetect.py")), "``snowboy_configuration[0]`` must be a Snowboy root directory containing ``snowboydetect.py``"
            for hot_word_file in snowboy_configuration[1]:
                assert os.path.isfile(hot_word_file), "``snowboy_configuration[1]`` must be a list of Snowboy hot word configuration files"

        seconds_per_buffer = float(source.CHUNK) / source.SAMPLE_RATE
        pause_buffer_count = int(math.ceil(self.pause_threshold / seconds_per_buffer))  # number of buffers of non-speaking audio during a phrase, before the phrase should be considered complete
        phrase_buffer_count = int(math.ceil(self.phrase_threshold / seconds_per_buffer))  # minimum number of buffers of speaking audio before we consider the speaking audio a phrase
        non_speaking_buffer_count = int(math.ceil(self.non_speaking_duration / seconds_per_buffer))  # maximum number of buffers of non-speaking audio to retain before and after a phrase

        # read audio input for phrases until there is a phrase that is long enough
        elapsed_time = 0  # number of seconds of audio read
        buffer = b""  # an empty buffer means that the stream has ended and there is no data left to read
        while True:
            frames = collections.deque()

            if snowboy_configuration is None:
                # store audio input until the phrase starts
                while True:
                    # handle waiting too long for phrase by raising an exception
                    elapsed_time += seconds_per_buffer
                    if timeout and elapsed_time > timeout:
                        raise SR.WaitTimeoutError("listening timed out while waiting for phrase to start")

                    buffer = source.stream.read(source.CHUNK)
                    if len(buffer) == 0: break  # reached end of the stream
                    frames.append(buffer)
                    if len(frames) > non_speaking_buffer_count:  # ensure we only keep the needed amount of non-speaking buffers
                        frames.popleft()

                    # detect whether speaking has started on audio input
                    energy = audioop.rms(buffer, source.SAMPLE_WIDTH)  # energy of the audio signal
                    if energy > self.energy_threshold: break

                    # dynamically adjust the energy threshold using asymmetric weighted average
                    if self.dynamic_energy_threshold:
                        damping = self.dynamic_energy_adjustment_damping ** seconds_per_buffer  # account for different chunk sizes and rates
                        target_energy = energy * self.dynamic_energy_ratio
                        self.energy_threshold = self.energy_threshold * damping + target_energy * (1 - damping)
            else:
                # read audio input until the hotword is said
                snowboy_location, snowboy_hot_word_files = snowboy_configuration
                buffer, delta_time = self.snowboy_wait_for_hot_word(snowboy_location, snowboy_hot_word_files, source, timeout)
                elapsed_time += delta_time
                if len(buffer) == 0: break  # reached end of the stream
                frames.append(buffer)

            # read audio input until the phrase ends
            pause_count, phrase_count = 0, 0
            phrase_start_time = elapsed_time
            while True:
                # handle phrase being too long by cutting off the audio
                elapsed_time += seconds_per_buffer
                if phrase_time_limit and elapsed_time - phrase_start_time > phrase_time_limit:
                    break

                if publisher and not self.is_speaking:
                    publisher.publish(Bool(data=True))
                self.is_speaking = True

                buffer = source.stream.read(source.CHUNK)
                if len(buffer) == 0: break  # reached end of the stream
                frames.append(buffer)
                phrase_count += 1

                # check if speaking has stopped for longer than the pause threshold on the audio input
                energy = audioop.rms(buffer, source.SAMPLE_WIDTH)  # unit energy of the audio signal within the buffer
                if energy > self.energy_threshold:
                    pause_count = 0
                    if energy > self.energy_threshold_loud and not self.is_speaking_loud:
                        self.is_speaking_loud = True
                        if publisher_loud:
                            publisher_loud.publish(Bool(data=True))
                else:
                    pause_count += 1
                if pause_count > pause_buffer_count:  # end of the phrase
                    break

            if publisher:
                publisher.publish(Bool(data=False))
            if publisher_loud and self.is_speaking_loud:
                publisher_loud.publish(Bool(data=False))
            self.is_speaking = False
            self.is_speaking_loud = False

            # check how long the detected phrase is, and retry listening if the phrase is too short
            phrase_count -= pause_count  # exclude the buffers for the pause before the phrase
            if phrase_count >= phrase_buffer_count or len(buffer) == 0: break  # phrase is long enough or we've reached the end of the stream, so stop listening

        # obtain frame data
        for i in range(pause_count - non_speaking_buffer_count): frames.pop()  # remove extra non-speaking frames at the end
        frame_data = b"".join(frames)

        return SR.AudioData(frame_data, source.SAMPLE_RATE, source.SAMPLE_WIDTH)

    def listen_in_background(self, source, callback, phrase_time_limit=None, publisher=None, publisher_loud=None):
        """
        Spawns a thread to repeatedly record phrases from ``source`` (an ``AudioSource`` instance) into an ``AudioData`` instance and call ``callback`` with that ``AudioData`` instance as soon as each phrase are detected.
        Returns a function object that, when called, requests that the background listener thread stop. The background thread is a daemon and will not stop the program from exiting if there are no other non-daemon threads. The function accepts one parameter, ``wait_for_stop``: if truthy, the function will wait for the background listener to stop before returning, otherwise it will return immediately and the background listener thread might still be running for a second or two afterwards. Additionally, if you are using a truthy value for ``wait_for_stop``, you must call the function from the same thread you originally called ``listen_in_background`` from.
        Phrase recognition uses the exact same mechanism as ``recognizer_instance.listen(source)``. The ``phrase_time_limit`` parameter works in the same way as the ``phrase_time_limit`` parameter for ``recognizer_instance.listen(source)``, as well.
        The ``callback`` parameter is a function that should accept two parameters - the ``recognizer_instance``, and an ``AudioData`` instance representing the captured audio. Note that ``callback`` function will be called from a non-main thread.
        """
        assert isinstance(source, SR.AudioSource), "Source must be an audio source"
        running = [True]

        def threaded_listen():
            with source as s:
                while running[0]:
                    try:  # listen for 1 second, then check again if the stop function has been called
                        audio = self.listen(s, 1, phrase_time_limit, publisher=publisher, publisher_loud=publisher_loud)
                    except SR.WaitTimeoutError:  # listening timed out, just try again
                        pass
                    else:
                        if running[0]: callback(self, audio)

        def stopper(wait_for_stop=True):
            running[0] = False
            if wait_for_stop:
                listener_thread.join()  # block until the background thread is done, which can take around 1 second

        listener_thread = threading.Thread(target=threaded_listen)
        listener_thread.daemon = True
        listener_thread.start()
        return stopper


class ROSAudio(SR.AudioSource):
    def __init__(self, topic_name="audio", depth=16, sample_rate=16000, chunk_size=1024, buffer_size=10240):
        assert buffer_size > chunk_size

        self.topic_name = topic_name
        self.buffer_size = buffer_size

        if depth == 8:
            self.SAMPLE_WIDTH = 1L
        elif depth == 16:
            self.SAMPLE_WIDTH = 2L
        elif depth == 32:
            self.SAMPLE_WIDTH = 4L
        else:
            raise ValueError("depth must be 8, 16 or 32")

        self.SAMPLE_RATE = sample_rate
        self.CHUNK = chunk_size

        self.stream = None

    def open(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None
        self.stream = ROSAudio.AudioStream(self.topic_name, self.buffer_size)
        return self

    def close(self):
        self.stream.close()
        self.stream = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    class AudioStream(object):
        def __init__(self, topic_name, buffer_size=10240):
            self.buffer_size = buffer_size
            self.lock = Lock()
            self.buffer = bytes()
            self.sub_audio = rospy.Subscriber(
                topic_name, AudioData, self.audio_cb)

        def read_once(self, size):
            with self.lock:
                buf = self.buffer[:size]
                self.buffer = self.buffer[size:]
                return buf

        def read(self, size):
            while not rospy.is_shutdown() and len(self.buffer) < size:
                rospy.sleep(0.001)
            return self.read_once(size)

        def close(self):
            try:
                self.sub_audio.unregister()
            except:
                pass
            self.buffer = bytes()

        def audio_cb(self, msg):
            with self.lock:
                self.buffer += bytes(msg.data)
                overflow = len(self.buffer) - self.buffer_size
                if overflow > 0:
                    self.buffer = self.buffer[overflow:]


class ROSSpeechRecognition(object):
    def __init__(self):
        self.default_duration = rospy.get_param("~duration", 10.0)
        self.engine = None
        self.recognizer = MyRecognizer(rospy.get_param("~rate_loud", 50.0))
        # self.recognizer = SR.Recognizer()
        self.audio = ROSAudio(topic_name="audio",
                              depth=rospy.get_param("~depth", 16),
                              sample_rate=rospy.get_param("~sample_rate", 16000))

        # initialize sound play client
        self.act_sound = actionlib.SimpleActionClient("sound_play", SoundRequestAction)
        if not self.act_sound.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Failed to find sound_play action. Disabled audio alert")
            self.act_sound = None
        self.signals = {
            "start": rospy.get_param("~start_signal",
                                     "/usr/share/sounds/ubuntu/stereo/bell.ogg"),
            "recognized": rospy.get_param("~recognized_signal",
                                          "/usr/share/sounds/ubuntu/stereo/button-toggle-on.ogg"),
            "success": rospy.get_param("~success_signal",
                                       "/usr/share/sounds/ubuntu/stereo/message-new-instant.ogg"),
            "timeout": rospy.get_param("~timeout_signal",
                                       "/usr/share/sounds/ubuntu/stereo/window-slide.ogg"),
        }

        self.dyn_srv = Server(Config, self.config_callback)

        self.stop_fn = None
        self.continuous = rospy.get_param("~continuous", False)
        if self.continuous:
            rospy.loginfo("Enabled continuous mode")
            self.pub = rospy.Publisher("/Tablet/voice",
                                       SpeechRecognitionCandidates,
                                       queue_size=1)
        else:
            self.srv = rospy.Service("speech_recognition",
                                     SpeechRecognition,
                                     self.speech_recognition_srv_cb)
        self.pub_is_speaking = rospy.Publisher("is_speaking",
                                               Bool,
                                               queue_size=1)
        self.pub_is_speaking_loud = rospy.Publisher("is_speaking_loud",
                                                    Bool,
                                                    queue_size=1)

    def config_callback(self, config, level):
        # config for engine
        self.language = config.language
        if self.engine != config.engine:
            self.args = {}
            self.engine = config.engine

        # config for adaptive thresholding
        self.dynamic_energy_threshold = config.dynamic_energy_threshold
        if self.dynamic_energy_threshold:
            config.energy_threshold = self.recognizer.energy_threshold
        else:
            self.recognizer.energy_threshold = config.energy_threshold
        self.recognizer.dynamic_energy_adjustment_damping = config.dynamic_energy_adjustment_damping
        self.recognizer.dynamic_energy_ratio = config.dynamic_energy_ratio

        # config for timeout
        if config.listen_timeout > 0.0:
            self.listen_timeout = config.listen_timeout
        else:
            self.listen_timeout = None
        if config.phrase_time_limit > 0.0:
            self.phrase_time_limit = config.phrase_time_limit
        else:
            self.phrase_time_limit = None
        if config.operation_timeout > 0.0:
            self.recognizer.operation_timeout = config.operation_timeout
        else:
            self.recognizer.operation_timeout = None

        # config for VAD
        if config.pause_threshold < config.non_speaking_duration:
            config.pause_threshold = config.non_speaking_duration
        self.recognizer.pause_threshold = config.pause_threshold
        self.recognizer.non_speaking_duration = config.non_speaking_duration
        self.recognizer.phrase_threshold = config.phrase_threshold

        return config

    def play_sound(self, key, timeout=5.0):
        if self.act_sound is None:
            return
        req = SoundRequest()
        req.sound = SoundRequest.PLAY_FILE
        req.command = SoundRequest.PLAY_ONCE
        if hasattr(SoundRequest, 'volume'): # volume is added from 0.3.0 https://github.com/ros-drivers/audio_common/commit/da9623414f381642e52f59701c09928c72a54be7#diff-fe2d85580f1ccfed4e23a608df44a7f7
            sound.volume = 1.0
        req.arg = self.signals[key]
        goal = SoundRequestGoal(sound_request=req)
        self.act_sound.send_goal_and_wait(goal, rospy.Duration(timeout))

    def recognize(self, audio):
        recog_func = None
        if self.engine == Config.SpeechRecognition_Google:
            if not self.args:
                self.args = {'key': rospy.get_param("~google_key", None)}
            recog_func = self.recognizer.recognize_google
        elif self.engine == Config.SpeechRecognition_GoogleCloud:
            if not self.args:
                credentials_path = rospy.get_param("~google_cloud_credentials_json", None)
                if credentials_path is not None:
                    with open(credentials_path) as j:
                        credentials_json = j.read()
                else:
                    credentials_json = None
                self.args = {'credentials_json': credentials_json,
                             'preferred_phrases': rospy.get_param('~google_cloud_preferred_phrases', None)}
            recog_func = self.recognizer.recognize_google_cloud
        elif self.engine == Config.SpeechRecognition_Sphinx:
            recog_func = self.recognizer.recognize_sphinx
        elif self.engine == Config.SpeechRecognition_Wit:
            recog_func = self.recognizer.recognize_wit
        elif self.engine == Config.SpeechRecognition_Bing:
            if not self.args:
                self.args = {'key': rospy.get_param("~bing_key")}
            recog_func = self.recognizer.recognize_bing
        elif self.engine == Config.SpeechRecognition_Houndify:
            recog_func = self.recognizer.recognize_houndify
        elif self.engine == Config.SpeechRecognition_IBM:
            recog_func = self.recognizer.recognize_ibm

        return recog_func(audio_data=audio, language=self.language, **self.args)

    def audio_cb(self, _, audio):
        try:
            rospy.logdebug("Waiting for result... (Sent %d bytes)" % len(audio.get_raw_data()))
            result = self.recognize(audio)
            rospy.loginfo("Result: %s" % result.encode('utf-8'))
            msg = SpeechRecognitionCandidates(transcript=[result])
            self.pub.publish(msg)
        except SR.UnknownValueError as e:
            if self.dynamic_energy_threshold:
                self.recognizer.adjust_for_ambient_noise(self.audio)
                rospy.loginfo("Updated energy threshold to %f" % self.recognizer.energy_threshold)
        except SR.RequestError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))

    def start_speech_recognition(self):
        if self.dynamic_energy_threshold:
            with self.audio as src:
                self.recognizer.adjust_for_ambient_noise(src)
                rospy.loginfo("Set minimum energy threshold to {}".format(
                    self.recognizer.energy_threshold))
        self.stop_fn = self.recognizer.listen_in_background(
            self.audio, self.audio_cb, phrase_time_limit=self.phrase_time_limit, publisher=self.pub_is_speaking, publisher_loud=self.pub_is_speaking_loud)
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        if self.stop_fn is not None:
            self.stop_fn()

    def speech_recognition_srv_cb(self, req):
        res = SpeechRecognitionResponse()

        duration = req.duration
        if duration <= 0.0:
            duration = self.default_duration

        with self.audio as src:
            if self.dynamic_energy_threshold:
                self.recognizer.adjust_for_ambient_noise(src)
                rospy.loginfo("Set minimum energy threshold to %f" % self.recognizer.energy_threshold)

            if not req.quiet:
                self.play_sound("start", 0.1)

            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration:
                rospy.loginfo("Waiting for speech...")
                try:
                    audio = self.recognizer.listen(
                        src, timeout=self.listen_timeout, phrase_time_limit=self.phrase_time_limit)
                except SR.WaitTimeoutError as e:
                    rospy.logwarn(e)
                    break
                if not req.quiet:
                    self.play_sound("recognized", 0.05)
                rospy.loginfo("Waiting for result... (Sent %d bytes)" % len(audio.get_raw_data()))

                try:
                    result = self.recognize(audio)
                    rospy.loginfo("Result: %s" % result.encode('utf-8'))
                    if not req.quiet:
                        self.play_sound("success", 0.1)
                    res.result = SpeechRecognitionCandidates(transcript=[result])
                    return res
                except SR.UnknownValueError:
                    if self.dynamic_energy_threshold:
                        self.recognizer.adjust_for_ambient_noise(src)
                        rospy.loginfo("Set minimum energy threshold to %f" % self.recognizer.energy_threshold)
                except SR.RequestError as e:
                    rospy.logerr("Failed to recognize: %s" % str(e))
                rospy.sleep(0.1)
                if rospy.is_shutdown():
                    break

            # Timeout
            if not req.quiet:
                self.play_sound("timeout", 0.1)
            return res

    def spin(self):
        if self.continuous:
            self.start_speech_recognition()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("speech_recognition")
    rec = ROSSpeechRecognition()
    rec.spin()
