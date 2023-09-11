/* *
 * This sample demonstrates handling intents from an Alexa skill using the Alexa Skills Kit SDK (v2).
 * Please visit https://alexa.design/cookbook for additional examples on implementing slots, dialog management,
 * session persistence, api calls, and more.
 * */
import Alexa from 'ask-sdk-core';
import http from 'http'

//////////////////////////////////////////////////////////////////////

function do_http_get(url) {

    return new Promise((resolve, reject) => {

        http.get(url, (response) => {

            let body = [];

            response.on('data', (data) => {
                body.push(data);
            });

            response.on('end', () => {
                resolve(body.toString());
            });

            response.on('error', (error) => {
                console.log(`HTTP ERROR: ${error}`);
                reject(error);
            });
        });
    });
}

//////////////////////////////////////////////////////////////////////

function distanceToPercent(d) {

    const min_dist = 120;
    const max_dist = 235;
    const range = max_dist - min_dist;
    if (d < min_dist) {
        d = min_dist
    }
    if (d > max_dist) {
        d = max_dist
    }
    let percent = (((max_dist - d) * 10 / range) | 0) * 10;
}

//////////////////////////////////////////////////////////////////////

async function getReading(url) {

    try {

        const result = await do_http_get(url);

        const data = JSON.parse(result);

        return {
            distance: data['distance'][0],
            time: data['time'][0],
            vbat: data['vbat'][0],
            rssi: data['rssi'][0]
        };

    } catch (err) {
        console.log(`getReading error: ${err}`);
    }

    return null;
}

//////////////////////////////////////////////////////////////////////

const LaunchRequestHandler = {

    canHandle(handlerInput) {

        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'LaunchRequest';
    },

    async handle(handlerInput) {

        var speakOutput = "ABC";

        let reading = await getReading('http://vibue.com:5002/readings?device=84f3eb536123&count=1');

        if (reading) {

            let percent = distanceToPercent(reading.distance);
            speakOutput = `The salt level is ${percent} percent`;

        } else {
            speakOutput = `I'm sorry, I can't get the salt level at the moment, please try later`;
        }

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

//////////////////////////////////////////////////////////////////////

const LevelHandler = {
    canHandle(handlerInput) {
        const request = handlerInput.requestEnvelope.request;
        return request.type === 'IntentRequest' &&
            request.intent.name === 'salty_intent';
    },
    async handle(handlerInput) {

        // get latest salt level reading and say it

        let text = "The salt level is all sort of crazy"
        return handlerInput.responseBuilder.speak(text).getResponse();
    }
};


//////////////////////////////////////////////////////////////////////

const HelloWorldIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'HelloWorldIntent';
    },
    handle(handlerInput) {
        const speakOutput = 'Hello World!';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};

//////////////////////////////////////////////////////////////////////

const HelpIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.HelpIntent';
    },
    handle(handlerInput) {
        const speakOutput = 'You can say hello to me! How can I help?';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

//////////////////////////////////////////////////////////////////////

const CancelAndStopIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && (Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.CancelIntent'
                || Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.StopIntent');
    },
    handle(handlerInput) {
        const speakOutput = 'Goodbye!';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .getResponse();
    }
};

//////////////////////////////////////////////////////////////////////

/* *
 * FallbackIntent triggers when a customer says something that doesn’t map to any intents in your skill
 * It must also be defined in the language model (if the locale supports it)
 * This handler can be safely added but will be ingnored in locales that do not support it yet 
 * */
const FallbackIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.FallbackIntent';
    },
    handle(handlerInput) {
        const speakOutput = 'Sorry, I don\'t know about that. Please try again.';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

//////////////////////////////////////////////////////////////////////

/* *
 * SessionEndedRequest notifies that a session was ended. This handler will be triggered when a currently open 
 * session is closed for one of the following reasons: 1) The user says "exit" or "quit". 2) The user does not 
 * respond or says something that does not match an intent defined in your voice model. 3) An error occurs 
 * */
const SessionEndedRequestHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'SessionEndedRequest';
    },
    handle(handlerInput) {
        console.log(`~~~~ Session ended: ${JSON.stringify(handlerInput.requestEnvelope)}`);
        // Any cleanup logic goes here.
        return handlerInput.responseBuilder.getResponse(); // notice we send an empty response
    }
};

//////////////////////////////////////////////////////////////////////

/* *
 * The intent reflector is used for interaction model testing and debugging.
 * It will simply repeat the intent the user said. You can create custom handlers for your intents 
 * by defining them above, then also adding them to the request handler chain below 
 * */

const IntentReflectorHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest';
    },
    handle(handlerInput) {
        const intentName = Alexa.getIntentName(handlerInput.requestEnvelope);
        const speakOutput = `You just triggered ${intentName}`;

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};

//////////////////////////////////////////////////////////////////////

/**
 * Generic error handling to capture any syntax or routing errors. If you receive an error
 * stating the request handler chain is not found, you have not implemented a handler for
 * the intent being invoked or included it in the skill builder below 
 * */
const ErrorHandler = {
    canHandle() {
        return true;
    },
    handle(handlerInput, error) {
        const speakOutput = 'Sorry, I had trouble doing what you asked. Please try again.';
        console.log(`~~~~ Error handled: ${JSON.stringify(error)}`);

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

export const handler = Alexa.SkillBuilders.custom()
    .addRequestHandlers(
        LaunchRequestHandler,
        LevelHandler,
        HelloWorldIntentHandler,
        HelpIntentHandler,
        CancelAndStopIntentHandler,
        FallbackIntentHandler,
        SessionEndedRequestHandler,
        IntentReflectorHandler)
    .addErrorHandlers(
        ErrorHandler)
    .withCustomUserAgent('sample/hello-world/v1.2')
    .lambda();
