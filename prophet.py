
import fbprophet  as fb
import matplotlib as mt
import pandas     as pd
import datetime   as dt


now = dt.datetime.now()
def calculate_time(row):
    return now + dt.timedelta(hours=row['id'])


def runProphet(filename, axis):

    #####################################################################
    # 
    # Load the data and produce a predicted time series with fbprophet

    # First load the data from the CSV file
    data = pd.read_csv(filename + '.csv')

    # Prepare the data for fbprophet. It requires 2 cols named 'ds' and 'y'
    data['ds'] = data.apply(calculate_time, axis=1)
    data['y']  = data[axis]

    print(data)

    # Now call fbprophet
    prophet = fb.Prophet()
    prophet.fit(data)

    # Make a future dataframe with 15 additional records to hold 15 predictions
    forecast = prophet.make_future_dataframe(periods=15)
    # and make predictions
    forecast = prophet.predict(forecast)
    print(forecast[['ds', 'yhat', 'yhat_lower', 'yhat_upper']])

    fig = prophet.plot(forecast)
    mt.pyplot.savefig('png/' + filename + '_' + axis + '_' + 'Series.png')


runProphet('moving_objects', 'x0')
sys.exit(0)
runProphet('moving_objects', 'x1')
runProphet('static_objects', 'x0')
runProphet('static_objects', 'x1')

