#!/usr/bin/env python3

import io
import os

import requests
# Imports the Google Cloud client library
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types

# Gazebo
# prefix = "http://10.16.103.133:8080/"
prefix = "http://10.16.104.100:8080/"


# prefix = "http://turtle1.athenian.org:8080/"


def move_robot(direction):
    try:
        return requests.get(prefix + direction)
    except BaseException as e:
        print(e)


def main():
    # Instantiates a client
    client = speech.SpeechClient()

    while (True):
        input("Hit return to give command")
        # os.system("say 'speak'")
        os.system("rec --rate 16k --channels=1 test.flac trim 0 1.5")

        # The name of the audio file to transcribe
        file_name = os.path.join(os.path.dirname(__file__) + '/test.flac')

        # Loads the audio into memory
        with io.open(file_name, 'rb') as audio_file:
            content = audio_file.read()
            audio = types.RecognitionAudio(content=content)

        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.FLAC,
            sample_rate_hertz=16000,
            language_code='en-US')

        # Detects speech in the audio file
        response = client.recognize(config, audio)

        for result in response.results:
            translation = result.alternatives[0].transcript
            print('Transcript: {}'.format(translation))
            print('Confidence: {}'.format(result.alternatives[0].confidence))

            if ("left" in translation):
                print("Send left command")
                resp = move_robot('left')
            elif ("right" in translation):
                print("Send right command")
                resp = move_robot('right')
            elif ("forward" in translation):
                print("Send forward command")
                resp = move_robot('forward')
            elif ("back" in translation):
                print("Send backward command")
                resp = move_robot('backward')
            elif ("stop" in translation):
                print("Send stop command")
                resp = move_robot('stop')


if __name__ == "__main__":
    main()
